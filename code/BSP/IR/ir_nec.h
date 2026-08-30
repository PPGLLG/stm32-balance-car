#ifndef _IR_NEC_H
#define _IR_NEC_H

#include <stdint.h>

/* ============ 红外 NEC 解码驱动 ============
 * 原理：HX1838 输出的是"脉宽调制"波形，0/1 编码在脉冲持续时间里。
 *       每个下降沿触发 EXTI，中断里读 TIM3 计数器，量"这个沿离上个沿多久"，
 *       按时间分类出帧头 / 0 / 1，一位位拼进 32 位变量，凑满就是键码。
 *
 * 前提（CubeMX 已配好）：
 *   - PA6 配置为 EXTI 下降沿 + 上拉
 *   - TIM3 时钟 1µs/格（Prescaler=71），计数器启动一次后一直跑
 *
 * 本模块不依赖 FreeRTOS，只做"时间 → 键码"的纯解码。
 * 通知任务、唤醒之类的事由上层（remote_task 桥接）负责。
 */
void    IR_NEC_Init(void);            /* 初始化：状态清零 + 启动 TIM3 */
void    IR_NEC_EdgeHandler(void);     /* 每个下降沿调用一次（EXTI 回调里）*/
uint8_t IR_NEC_KeyReady(void);        /* 返回 1 = 键码已解码完成（未清）*/
uint8_t IR_NEC_GetKey(void);          /* 读出键码并清就绪标志（读一次）*/

#endif /* _IR_NEC_H */
