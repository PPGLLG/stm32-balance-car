#ifndef _SOFT_I2C_H
#define _SOFT_I2C_H

#include <stdint.h>

/* ============ 软件模拟 I2C（bit-bang，PB8=SCL / PB9=SDA）============
 * 为什么用软件：STM32F1 的 I2C1 硬件外设在长数据串(≥128字节)上会花屏/丢数据，
 *   软件 I2C 用 GPIO 手动模拟时序，慢速+慢翻转，波形干净，实测稳定。
 * 地址用 8 位形式（与 HAL 一致）：OLED=0x78，MPU6050=0xD0。
 *
 * 用法：
 *   soft_i2c_init();                          // 上电调用一次（幂等）
 *   soft_i2c_write(0x78, buf, n);             // 发 地址+数据
 *   soft_i2c_read (0xD0|1, buf, n);           // 读 n 字节（MPU 用，暂未接线）
 *
 * 注意：本模块不内置互斥锁，调用方（oled.c / mpu6050.c）自行拿 i2c_mutex。
 */

void    soft_i2c_init(void);                              /* PB8/PB9 → 开漏输出 */
uint8_t soft_i2c_write(uint8_t addr8, const uint8_t *data, uint16_t len);  /* 返回0=全应答 */
uint8_t soft_i2c_read (uint8_t addr8, uint8_t *buf, uint16_t len);         /* 返回0=成功 */

#endif /* _SOFT_I2C_H */
