#include "FreeRTOS.h"      // 内核头（ulTaskNotifyTake 等的前置）
#include "task.h"          // 任务 API（xTaskGetCurrentTaskHandle 等）
#include "remote_task.h"  // 遥控任务头文件
#include "main.h"          /* MPU_INT_Pin（PB14）*/
#include "queue.h"
#include "ir_nec.h"      /* IR_NEC_EdgeHandler() */
#include "ring_buf.h"    /* RingBuf_t */
#include "translate.h"   /* translate_ir / translate_wifi */
#include "usart.h"       /* huart1/huart2 */
#include "esp01.h"       /* ESP01_Init（WiFi AT 配置）*/
static TaskHandle_t remoteTaskHandle;//遥控任务句柄
QueueHandle_t remoteQueueHandle; //遥控数据邮箱句柄
static uint8_t   wifi_rx_data[64];   /* WiFi 环形缓冲数据区 */
static RingBuf_t wifi_rx;            /* WiFi 环形缓冲 */
static uint8_t   wifi_rx_byte;       /* WiFi 串口单字节接收缓冲 */
static uint8_t   current_source = 0; /* 当前控制源：1=红外 2=WiFi 0=无 */
static uint32_t  last_command_tick = 0; /* 失联保护：最后收到有效命令的时刻 */
void remoteTask(void *argument)
{
  (void)argument;

  remoteTaskHandle = xTaskGetCurrentTaskHandle();  /* 保存自己的句柄，供中断回调 */

  //初始化
  IR_NEC_Init(); //红外接收初始化
  RingBuf_Init(&wifi_rx, wifi_rx_data, sizeof(wifi_rx_data)); //WiFi环形缓冲
  ESP01_Init(); //WiFi模块 AT 配置（AP + TCP server）
  HAL_UART_Receive_IT(&huart2, &wifi_rx_byte, 1); //WiFi串口启动逐字节接收

  RemoteData_t remoteData = { .speed = 0, .turn = 0, .angle = 3.0f }; /* 目标值，平衡角默认 3.0（与 control.c 的 g_angle_aim 一致）*/
  last_command_tick = xTaskGetTickCount(); /* 失联保护计时起点 */

  while(1){
    uint32_t notificationValue;//接收通知值
    xTaskNotifyWait(pdFALSE, pdTRUE, &notificationValue, pdMS_TO_TICKS(100));//等待通知,超时为100ms
    if(notificationValue & 0x01) {
        //如果第0位为1，处理红外数据
        if (translate_ir(&remoteData)) {                     /* 键码 → 命令 */
            current_source = 1;                              /* 记录控制源 */
            last_command_tick = xTaskGetTickCount();         /* 失联保护：更新最后命令时刻 */
            xQueueOverwrite(remoteQueueHandle, &remoteData); /* 有命令才写邮箱 */
        }
    }
    if(notificationValue & 0x02) {
        //如果第1位为1，处理wifi数据
        if (translate_wifi(&remoteData, &wifi_rx)) {         /* 攒行 → 命令 */
            current_source = 2;                              /* 记录控制源 */
            last_command_tick = xTaskGetTickCount();         /* 失联保护：更新最后命令时刻 */
            xQueueOverwrite(remoteQueueHandle, &remoteData); /* 有命令才写邮箱 */
        }
    }

    /* 失联保护：>500ms 没收到有效命令 → 速度/转向清零（平衡角保留）*/
    if (xTaskGetTickCount() - last_command_tick > 500) {
        remoteData.speed = 0;
        remoteData.turn  = 0;
        xQueueOverwrite(remoteQueueHandle, &remoteData);
    }

    /* WiFi 波形转发：control 任务准备好数据后置 ready，这里发给 Vofa（阻塞发送放本任务，不拖慢控制环）*/
    if (g_vofa_ready) {
        g_vofa_ready = 0;
        wifi_vofa_send(g_vofa_line, g_vofa_line_len);
    }
  }
}

/* 红外桥接：被 control_task 的 EXTI 回调转发进来（ISR 上下文）
 * 只做解码 + 有有效键码才置位唤醒，FreeRTOS 通知放这里，
 * 让 BSP/IR 保持纯解码、不碰任务句柄。 */
void IR_Edge_FromIsr(void)
{
    IR_NEC_EdgeHandler();                    /* 解码一步 */
    if (IR_NEC_KeyReady()) {                 /* 解出有效键码才唤醒 */
        if (remoteTaskHandle != NULL) {      /* 遥控任务被禁 → 跳过（防空指针）*/
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(remoteTaskHandle, 0x01, eSetBits, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* 当前控制源（显示用）：1=红外 2=WiFi 0=无 */
uint8_t remote_get_source(void)
{
    return current_source;
}

/* WiFi 串口接收回调（覆盖 HAL 弱函数）：USART2 每收一个字节进一次 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* ① 字节进环形缓冲（满则丢弃，RingBuf_Write 返回 0）*/
        RingBuf_Write(&wifi_rx, wifi_rx_byte);

        /* ② 置 WiFi 通知位(bit1=0x02) + 唤醒遥控任务 */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(remoteTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        /* ③ 重新武装：继续收下一个字节 */
        HAL_UART_Receive_IT(&huart2, &wifi_rx_byte, 1);
    }
}
