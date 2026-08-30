#include "esp01.h"
#include "usart.h"       /* huart2 */
#include "FreeRTOS.h"
#include "task.h"        /* vTaskDelay */

void ESP01_Init(void)
{
    /* 依次发 AT 命令配置 ESP。配好后手机连 ESP 热点，TCP 数据会以 +IPD 帧从 USART2 进来。
     * 用 sizeof-1 传字符串长度（不依赖 strlen/libc）。*/
    static const char at0[] = "ATE0\r\n";               /* 关闭回显，防止 AT+CIPSEND 被误解析成 A 命令 */
    static const char at1[] = "AT+CWMODE=2\r\n";        /* 设成 AP 模式 */
    static const char at2[] = "AT+CIPMUX=1\r\n";        /* 开启多连接 */
    static const char at3[] = "AT+CIPSERVER=1,333\r\n"; /* 起 TCP 服务器，端口 333 */

    HAL_UART_Transmit(&huart2, (uint8_t *)at0, sizeof(at0) - 1, 100);
    vTaskDelay(pdMS_TO_TICKS(300));                     /* 等 ESP 处理完上一条 */

    HAL_UART_Transmit(&huart2, (uint8_t *)at1, sizeof(at1) - 1, 100);
    vTaskDelay(pdMS_TO_TICKS(300));                     /* 等 ESP 处理完上一条 */

    HAL_UART_Transmit(&huart2, (uint8_t *)at2, sizeof(at2) - 1, 100);
    vTaskDelay(pdMS_TO_TICKS(300));

    HAL_UART_Transmit(&huart2, (uint8_t *)at3, sizeof(at3) - 1, 100);
    vTaskDelay(pdMS_TO_TICKS(300));
}
