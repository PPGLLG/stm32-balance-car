#ifndef _OLED_H
#define _OLED_H

#include <stdint.h>

/* ============ SSD1306 OLED 驱动（I2C1，地址 0x3C）============
 * 用法：
 *   OLED_Init();                          // 上电初始化一次
 *   画内容（改帧缓冲）→ OLED_Display();   // 一次性推屏
 *   或 OLED_DisplayPage(page)             // 按页刷新，分散 I2C 占用
 *   坐标 (0,0) 在左上角，x 向右，y 向下
 *
 * OLED 和 MPU6050 共用 I2C1 总线，I2C 互斥锁由 i2c_mutex 保证。
 */

#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_I2C_ADDR 0x78     /* F1 HAL 用 8 位地址（7位 0x3C << 1）*/

void OLED_Init(void);
void OLED_Clear(void);                       /* 清空帧缓冲（还没推屏）*/
void OLED_Display(void);                     /* 把帧缓冲一次性推到屏幕 */
void OLED_DisplayPage(uint8_t page);         /* 只刷新某一页，用于分散 I2C 占用 */
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on);
void OLED_DrawChar(uint8_t x, uint8_t y, char c);            /* 6x8 字符 */
void OLED_DrawString(uint8_t x, uint8_t y, const char *str); /* 6x8 字符串 */

#endif /* _OLED_H */
