#include "mpu6050.h"
#include "main.h"        /* HAL_Delay 等 */
#include "soft_i2c.h"    /* 软件 I2C（F1 硬件 I2C 长数据串会花屏，MPU6050 一并改用）*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "i2c_mutex.h"   /* I2C1 互斥锁（OLED 共用总线）*/

/* ===== MPU6050 寄存器地址（私有，只有本文件用）===== */
#define MPU6050_ADDR       0xD0    /* 8位地址 = 7位地址0x68 << 1（bit0=R/W位由软I2C处理）*/
#define WHO_AM_I_REG       0x75    /* 设备 ID 寄存器，应读回 0x68 */
#define SMPLRT_DIV_REG     0x19    /* 采样率分频：采样率 = 1kHz/(1+此值) */
#define CONFIG_REG         0x1A    /* DLPF 低通滤波配置 */
#define GYRO_CONFIG_REG    0x1B    /* 陀螺仪量程配置 */
#define ACCEL_CONFIG_REG   0x1C    /* 加速度计量程配置 */
#define ACCEL_XOUT_H_REG   0x3B    /* 加速度/陀螺仪数据块起始寄存器 */
#define PWR_MGMT_1_REG     0x6B    /* 电源管理寄存器1（写0唤醒）*/
#define INT_EN_REG         0x38    /* 中断使能寄存器（写0x01=开数据就绪中断）*/

mpu6050_data_t g_mpu;   /* 全局变量定义（类型来自 mpu6050.h）*/
uint8_t g_mpu_ok = 0;   /* MPU 初始化成功标志，WHO_AM_I 通过后置 1 */
float g_gyro_offset[3] = {0.0f, 0.0f, 0.0f};  /* 陀螺零偏（LSB），开机校准 */

/* 开机静止校准：采样 100 次陀螺求平均作为零偏。
 * ⚠️ 要求调用期间车完全静止（上电后约 1 秒别动），否则零偏不准 */
void MPU6050_CalibrateGyro(void)
{
    int32_t sum[3] = {0, 0, 0};
    uint16_t ok_count = 0;

    for (uint16_t i = 0; i < 100; i++) {
        if (MPU6050_Read()) {
            sum[0] += g_mpu.gyro[0];
            sum[1] += g_mpu.gyro[1];
            sum[2] += g_mpu.gyro[2];
            ok_count++;
        }
        HAL_Delay(10);
    }
    if (ok_count > 0) {
        g_gyro_offset[0] = (float)sum[0] / ok_count;
        g_gyro_offset[1] = (float)sum[1] / ok_count;
        g_gyro_offset[2] = (float)sum[2] / ok_count;
    }
}

/* 写单个寄存器：一个完整 I2C 事务（寄存器地址 + 1字节数据）*/
static void mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    soft_i2c_write(MPU6050_ADDR, buf, 2);
    xSemaphoreGive(i2c_mutex);
}

/* 读连续寄存器：先写寄存器指针，再连读 len 字节。
 * ⚠️ "写指针 + 读数据"必须在一个锁里：两步之间若释放锁，
 *    OLED 任务可能插进总线，打断 MPU 内部寄存器指针，读到错数据。 */
static uint8_t mpu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len, TickType_t timeout)
{
    /* timeout=0：总线被 OLED 占用时立即放弃本次（控制环不死等）；init 时传 portMAX_DELAY */
    if (xSemaphoreTake(i2c_mutex, timeout) != pdPASS) return 0;
    soft_i2c_write(MPU6050_ADDR, &reg, 1);   /* ① 先指向起始寄存器 */
    soft_i2c_read (MPU6050_ADDR, buf, len);  /* ② 连读 len 字节 */
    xSemaphoreGive(i2c_mutex);
    return 1;
}

void MPU6050_Init(void){
    //初始化MPU6050
    uint8_t check = 0;
    soft_i2c_init();   /* 幂等：PB8/PB9 配成开漏输出（OLED_Init 也会调，顺序无关）*/
    //检查MPU6050是否连接成功（WHO_AM_I 返回设备ID 0x68）
    mpu_read_regs(WHO_AM_I_REG, &check, 1, portMAX_DELAY);
    if(check == 0x68){
        //设置加速度计范围为±2g
        mpu_write_reg(ACCEL_CONFIG_REG, 0x00);
        //设置陀螺仪范围为±2000°/s（0x18。原±250°/s在车快速倒时饱和，角度滞后，见mpu6050.h说明）
        mpu_write_reg(GYRO_CONFIG_REG, 0x18);
        //设置采样率为250Hz（dt=0.004，与 filter.c 的 KALMAN_DT 一致；原来100Hz+42Hz滤波延迟太大）
        mpu_write_reg(SMPLRT_DIV_REG, 0x03);
        //设置低通滤波器为256Hz（原来42Hz引入~10ms相位延迟，是"车体和轮子不同步"的主因之一）
        mpu_write_reg(CONFIG_REG, 0x00);
        //唤醒MPU6050
        mpu_write_reg(PWR_MGMT_1_REG, 0x00);
        //数据就绪中断已弃用：控制任务改固定5ms软件唤醒，不再用INT。
        //关掉后 MPU 的 INT 引脚保持低电平，PB14 不再产生下降沿（EXTI 彻底安静）
        mpu_write_reg(INT_EN_REG, 0x00);
        g_mpu_ok = 1;   /* 全部配置成功才置位 */
    }
}

uint8_t MPU6050_Read(void)
{
    uint8_t buf[14];              /* 临时接收区 */

    /* "写0x3B指针 + 连读14字节"是原子事务，锁在 mpu_read_regs 内部跨两步持有。
     * timeout=1ms：OLED 刷新占用 I2C 时短暂等待，避免角度长时间冻结 */
    if (!mpu_read_regs(ACCEL_XOUT_H_REG, buf, 14, pdMS_TO_TICKS(1))) return 0;

    g_mpu.acc[0]  = (int16_t)((buf[0]  << 8) | buf[1]);   /* X 加速度 */
    g_mpu.acc[1]  = (int16_t)((buf[2]  << 8) | buf[3]);   /* Y */
    g_mpu.acc[2]  = (int16_t)((buf[4]  << 8) | buf[5]);   /* Z */
    /* buf[6],buf[7] 是温度，先不管 */
    g_mpu.gyro[0] = (int16_t)((buf[8]  << 8) | buf[9]);   /* X 陀螺 */
    g_mpu.gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);  /* Y */
    g_mpu.gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);  /* Z */
    return 1;
}
