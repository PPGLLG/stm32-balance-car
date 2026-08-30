#include "filter.h"
#include "mpu6050.h"        /* g_mpu + GYRO_SENSITIVITY */
#include <math.h>           /* atan2f */

#define RAD_TO_DEG  57.29578f   /* 弧度→度 */
#define FILTER_DT   0.005f       /* 采样周期：5ms（控制周期5ms，★必须与实际一致！）*/

Kalman_t g_kalman;   /* 保留旧命名，实际使用互补滤波 */
float g_angle;

void kalman_init(void)
{
    /* 互补滤波初始化：保留旧 Kalman 状态结构，但只使用 angle 字段 */
    g_kalman.angle = 0.0f;
    g_kalman.bias  = 0.0f;
    g_kalman.P[0][0] = 1.0f;
    g_kalman.P[0][1] = 0.0f;
    g_kalman.P[1][0] = 0.0f;
    g_kalman.P[1][1] = 1.0f;
}

void filter_update(void)
{
    /* 互补滤波：
     * 动态由陀螺积分即时提供，加速度计只做慢漂移校正。 */
    float accel_angle = atan2f((float)g_mpu.acc[0], (float)g_mpu.acc[2]) * RAD_TO_DEG;

    /* 陀螺 → °/s：先减开机校准的零偏，再 ÷ 灵敏度 */
    float gyro_rate = -((float)g_mpu.gyro[1] - g_gyro_offset[1]) / GYRO_SENSITIVITY;

    g_angle = 0.005f * accel_angle + 0.995f * (g_angle + gyro_rate * FILTER_DT);
    /* 加速度计权重 0.005，陀螺积分权重 0.995 */
}
