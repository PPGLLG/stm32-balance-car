#ifndef _REMOTE_TASK_H          // ← 防止重复包含（必须）
#define _REMOTE_TASK_H
#include "FreeRTOS.h"      // 内核头（ulTaskNotifyTake 等的前置）
#include "queue.h"
void remoteTask(void *argument);
void IR_Edge_FromIsr(void);   /* 红外 EXTI 桥接（control_task 回调转发过来，ISR 上下文）*/
uint8_t remote_get_source(void);  /* 当前控制源：1=红外 2=WiFi 0=无 */
extern QueueHandle_t remoteQueueHandle; //遥控数据邮箱句柄
typedef struct {
    float speed;  // 速度
    float turn;   // 转向
    float angle;  // 平衡角

} RemoteData_t;

#endif // _REMOTE_TASK_H