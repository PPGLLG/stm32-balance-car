#ifndef _MPU6050_H          // ← 防止重复包含（必须）
#define _MPU6050_H

#include <stdint.h>         // int16_t

/* 灵敏度宏（★ 必须和 MPU6050_Init 里的量程一致！）*/
#define GYRO_SENSITIVITY    16.4f     /* ±2000°/s 量程 → 16.4 LSB/(°/s)。
                                         原±250°/s(131)在车快速倒时饱和(>250°/s)，
                                         角度积分追不上真实角度→软倒后巨大甩动 */
#define ACCEL_SENSITIVITY   16384.0f   /* ±2g 量程 → 16384 LSB/g */

/* 原始数据结构体 */
typedef struct {
    int16_t acc[3];    /* acc[0]=X  acc[1]=Y  acc[2]=Z */
    int16_t gyro[3];   /* gyro[0]=X gyro[1]=Y gyro[2]=Z */
} mpu6050_data_t;

extern mpu6050_data_t g_mpu;   /* 全局，原始数据存在这里 */
extern uint8_t g_mpu_ok;       /* MPU6050 初始化成功标志（0=失败，1=成功）*/
extern float g_gyro_offset[3];  /* 陀螺零偏（LSB），开机静止校准得到，使用前要减去 */

void MPU6050_Init(void);//初始化MPU6050,修改.h文件中的宏定义可以修改mpu6050的采样率和量程
uint8_t MPU6050_Read(void);//读取MPU6050数据,储存在结构体g_mpu中；返回1=读到新数据，0=总线忙跳过
void MPU6050_CalibrateGyro(void);//开机静止校准陀螺零偏（要求上电后车静止约1秒）
#endif // _MPU6050_H
