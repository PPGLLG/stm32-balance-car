#ifndef _filter_H          // ← 防止重复包含（必须）
#define _filter_H

#include <stdint.h>

/* 滤波状态（保留旧 Kalman 命名，当前实现为互补滤波） */
typedef struct {
    float Q_angle;      /* 加速度计修正权重相关参数 */
    float Q_bias;       /* 陀螺零漂相关参数 */
    float R_measure;    /* 测量噪声相关参数 */
    float angle;        /* 融合后的角度（内部状态）*/
    float bias;         /* 零漂估计（内部状态）*/
    float P[2][2];      /* 协方差矩阵（内部状态）*/
} Kalman_t;

extern Kalman_t g_kalman;   /* 滤波状态（初始化时用）*/
extern float g_angle;       /* ★ 融合后的俯仰角（度），控制任务读这个 */

void kalman_init(void);     /* 开机初始化一次 */
void filter_update(void);   /* 每次控制周期调用：互补滤波更新 g_angle */

#endif // _filter_H
