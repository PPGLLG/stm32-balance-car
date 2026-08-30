#include "FreeRTOS.h"      // 内核头（vTaskDelayUntil 等的前置）
#include "task.h"          // 任务 API（xTaskGetTickCount 等）
#include "control_task.h"  // 控制任务头文件
#include "control.h"      // 控制算法头文件
#include "filter.h"       // 滤波算法头文件
#include "bsp_motor.h"     /* PWMinit */
#include "mpu6050.h"       /* MPU6050_Init / MPU6050_Read */
#include "main.h"          /* MPU_INT_Pin（PB14）*/
#include "remote_task.h"   /* IR_Edge_FromIsr() */
#include "translate.h"     /* g_vofa_line 等（WiFi 波形共享缓冲）*/

/* Vofa+ 波形调试开关：1=开（调参时），0=关（省CPU和Flash）*/
#define ENABLE_VOFA_DEBUG  1

#if ENABLE_VOFA_DEBUG
/* 数值 → 字符串（dec 位小数），避免 printf %f 拉入大块浮点格式化代码 */
static void fmt_num(char *buf, float val, uint8_t dec)
{
    char tmp[14];
    uint8_t n = 0, i;
    uint32_t whole, frac = 0, p = 1;
    uint8_t neg = 0;

    if (val < 0) { neg = 1; val = -val; }
    whole = (uint32_t)val;
    if (dec) {
        float f = val - (float)whole;
        for (i = 0; i < dec; i++) f *= 10.0f;
        frac = (uint32_t)(f + 0.5f);
        for (i = 0; i < dec; i++) p *= 10;
        if (frac >= p) { frac = 0; whole++; }
    }
    do { tmp[n++] = (char)('0' + whole % 10); whole /= 10; } while (whole);
    if (neg) tmp[n++] = '-';
    for (i = n; i > 0; i--) *buf++ = tmp[i - 1];
    if (dec) {
        *buf++ = '.';
        for (i = 0; i < dec; i++) { p /= 10; *buf++ = (char)('0' + (frac / p) % 10); }
    }
    *buf = '\0';
}

/* 把格式化好的数值追加进缓冲，末尾加 ',' 分隔（FireWater：逗号分隔） */
static void append_num(char *buf, uint16_t *pos, float val, uint8_t dec)
{
    char tmp[12];
    uint8_t i = 0;
    fmt_num(tmp, val, dec);
    while (tmp[i]) buf[(*pos)++] = tmp[i++];
    buf[(*pos)++] = ',';
}
#endif /* ENABLE_VOFA_DEBUG */

/* 耗时自测（DWT 周期计数，72MHz → 1 周期 = 1/72µs）*/
static uint32_t g_mpu_us  = 0;   /* 读MPU+滤波耗时（µs）*/
static uint32_t g_loop_us = 0;   /* 控制周期执行总耗时（µs），接近10000=周期被拉长 */

/**
  * 在这个任务中实现了对小车的控制逻辑，该任务会周期性地读取传感器数据，调用滤波算法与模态解算，经过pid控制器计算出电机的控制量，最后通过串口发送给电机驱动板，实现对小车的运动控制。
  *使用了：pid，滤波，模态解算，串口，MPU6050
  * 无返回值
  */

void controlTask(void *argument)
{
  (void)argument;              /* 未使用参数，避免编译警告 */
  control_init();  /* 初始化 PID 参数 */
  kalman_init();  /* 初始化卡尔曼滤波器 */
  PWMinit();  /* 初始化电机 PWM 输出 */
  MPU6050_Init();  /* 初始化 MPU6050 */
  MPU6050_CalibrateGyro();  /* 开机静止校准陀螺零偏（⚠️ 期间约1秒，车别动！）*/

  /* DWT 硬件周期计数器（测耗时用）*/
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  TickType_t last_wake = xTaskGetTickCount();  /* 固定 5ms 唤醒的时间基准（缩短延迟）*/
  while(1){
    /* 固定 5ms 软件唤醒（与 KALMAN_DT=0.005 一致），缩短系统延迟 */
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
    uint32_t t_loop0 = DWT->CYCCNT;

    /* ① 读 MPU + 互补滤波：只有真正读到新数据才更新角度，
     *    避免 OLED 占用 I2C 时用旧数据错误积分导致抖动 */
    if (MPU6050_Read()) {
        filter_update();
    }
    g_mpu_us = (DWT->CYCCNT - t_loop0) / 72;   /* 读MPU+滤波耗时（µs）*/

    /* ①.5 从遥控邮箱取最新目标值（0超时不阻塞；邮箱空则保持原值）。
     *     WiFi 的 A/S/T 命令走邮箱——之前此段被删，导致 WiFi 调平衡角失效 */
    if (remoteQueueHandle != NULL) {
        RemoteData_t cmd;
        if (xQueuePeek(remoteQueueHandle, &cmd, 0) == pdPASS) {
            g_speed_target = cmd.speed;  /* 写入最终目标，由速度外环斜坡逼近 */
            g_turn_target  = cmd.turn;   /* 写入最终目标，由转向斜坡逼近 */
            g_angle_aim    = cmd.angle;
        }
    }

    /* ② 串级控制：速度外环每 4 个周期（20ms）更新一次，
     *    角度内环和输出每个控制周期（5ms）都更新 */
    static uint8_t speed_div = 0;
    if (++speed_div >= 4) {
        speed_div = 0;
        control_speed_update();
    }
    control_angle_update();
    control_turn();
    control_output();
    g_loop_us = (DWT->CYCCNT - t_loop0) / 72;   /* 本周期执行总耗时（µs）*/

#if ENABLE_VOFA_DEBUG
    /* 每 10 个周期发一次（≈10Hz）。FireWater 8通道：
     * 目标角度,实际角度,目标速度,实际速度,速度环输出,共同PWM,MPU耗时,循环耗时 */
    {
        static uint8_t div = 0;
        char line[96];
        uint16_t pos = 0;
        if (++div >= 10) {
            div = 0;
            append_num(line, &pos, g_angle_aim,          2);   /* CH0 目标俯仰角 */
            append_num(line, &pos, g_angle,               2);   /* CH1 实际俯仰角 */
            append_num(line, &pos, g_speed_aim,           0);   /* CH2 目标速度 */
            append_num(line, &pos, control_get_speed(),   0);   /* CH3 实际速度 */
            append_num(line, &pos, control_get_speed_out(), 2); /* CH4 速度环输出 */
            append_num(line, &pos, control_get_pwm(),     3);   /* CH5 共同PWM */
            append_num(line, &pos, (float)g_mpu_us,       0);   /* CH6 读MPU+滤波耗时(µs) */
            append_num(line, &pos, (float)g_loop_us,      0);   /* CH7 周期执行耗时(µs) */
            line[pos - 1] = '\n';          /* 最后一个','换成换行（FireWater每帧以换行结束）*/
            for (uint16_t i = 0; i < pos; i++) g_vofa_line[i] = line[i];
            g_vofa_line_len = pos;
            g_vofa_ready = 1;              /* 通知 remote 任务转发 */
        }
    }
#endif /* ENABLE_VOFA_DEBUG */
  }
}

/* EXTI 回调（覆盖 HAL 弱函数）：只服务红外遥控。
 * MPU_INT_Pin（PB14）已弃用：控制任务改固定 5ms 软件唤醒，不依赖 MPU 中断。
 * （EXTI 配置仍在 gpio.c/stm32f1xx_it.c，但 MPU 侧 INT_EN 已关，PB14 不再触发）*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IR_Pin)   /* PA6 红外 */
    {
        IR_Edge_FromIsr();
    }
}
