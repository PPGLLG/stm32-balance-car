#ifndef _I2C_MUTEX_H
#define _I2C_MUTEX_H

#include "FreeRTOS.h"
#include "semphr.h"

/* ============ I2C1 互斥锁 ============
 * OLED 和 MPU6050 共用 I2C1 总线：
 *   - 控制任务读 MPU6050 → I2C1
 *   - 显示任务写 OLED    → I2C1
 * 任何 I2C1 事务前必须先 Take，完整事务后 Give，防止两个任务绞总线。
 * 用互斥锁（带优先级继承），防止"显示持锁被遥控饿死、控制等不到"。
 *
 * 创建：freertos.c 的 MX_FREERTOS_Init（调度器启动前）
 */
extern SemaphoreHandle_t i2c_mutex;

#endif /* _I2C_MUTEX_H */
