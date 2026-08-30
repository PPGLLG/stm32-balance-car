#ifndef _ESP01_H
#define _ESP01_H

/* ============ ESP-01S WiFi 驱动 ============
 * 通过 AT 指令配置成 AP 模式 + TCP 服务器。
 * 手机连 ESP 热点 → TCP 发数据 → ESP 吐 +IPD 帧 → USART2 → 环形缓冲 → translate 剥壳解析
 */

void ESP01_Init(void);   /* 配置 ESP：AP 模式 + TCP 服务器（一次性，初始化时调）*/

#endif /* _ESP01_H */
