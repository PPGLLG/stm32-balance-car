#ifndef _TRANSLATE_H
#define _TRANSLATE_H

#include "remote_task.h"   /* RemoteData_t */
#include "ring_buf.h"      /* RingBuf_t */

/* ============ 翻译层：把各源的数据变成小车命令 ============
 * 每个源一个入口，输出统一为 RemoteData_t（速度/转向/平衡角）。
 *
 * 用法约定：
 *   - data 由调用方（remote_task）初始化为 0，并长期持有
 *   - 翻译只更新它该管的字段，不动的字段保持原值
 *   - 返回 1 = 产生了命令（data 已更新）；0 = 无命令/忽略
 *   - remote_task 在返回 1 时才写邮箱 */

uint8_t translate_ir(RemoteData_t *data);                  /* 红外：键码 → 命令 */
uint8_t translate_wifi(RemoteData_t *data, RingBuf_t *rb); /* WiFi：攒行 → 解析 */

/* ============ WiFi 波形发送（Vofa 经 ESP01 TCP 无线看波形）============ */
extern volatile uint8_t  g_vofa_ready;     /* control 置 1 = 有新波形数据待发 */
extern volatile uint16_t g_vofa_line_len;  /* 待发数据长度 */
extern char              g_vofa_line[96];  /* 待发数据缓冲 */
void wifi_vofa_send(const char *data, uint16_t len); /* 广播发给所有已连接客户端 */

#endif /* _TRANSLATE_H */
