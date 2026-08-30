#include "oled.h"
#include "font.h"
#include "soft_i2c.h"   /* 软件 I2C（F1 硬件 I2C 长数据串会花屏，改用软件模拟）*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "i2c_mutex.h"  /* I2C1 互斥锁（MPU6050 共用总线）*/

/* 帧缓冲：128×64，每行8个像素打包成一个字节，共 128×64/8 = 1024 字节 */
static uint8_t framebuffer[OLED_WIDTH * OLED_HEIGHT / 8];

/* 写命令（I2C 前导 0x00 = 命令），整个事务拿 I2C 锁 */
static void OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    soft_i2c_write(OLED_I2C_ADDR, buf, 2);
    xSemaphoreGive(i2c_mutex);
}

/* 写一页数据（I2C 前导 0x40 = 数据）。
 * 拆成小块发送，每次只占很短时间的 I2C 锁，避免长时间阻塞 MPU 读取。 */
#define OLED_I2C_CHUNK  16

static void OLED_WriteData(const uint8_t *data, uint16_t len)
{
    uint8_t buf[OLED_I2C_CHUNK + 1];
    uint16_t offset = 0;

    while (offset < len) {
        uint16_t chunk = len - offset;
        if (chunk > OLED_I2C_CHUNK) chunk = OLED_I2C_CHUNK;

        buf[0] = 0x40;
        for (uint16_t i = 0; i < chunk; i++) {
            buf[i + 1] = data[offset + i];
        }

        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        soft_i2c_write(OLED_I2C_ADDR, buf, chunk + 1);
        xSemaphoreGive(i2c_mutex);

        offset += chunk;
    }
}

void OLED_Init(void)
{
    soft_i2c_init();   /* 一次性配置 PB8/PB9 为开漏输出（幂等，可反复调用）*/
    static const uint8_t init_cmds[] = {
        0xAE, /* 关显示 */
        0x20, 0x00, /* 水平寻址模式 */
        0xB0, /* 页起始地址=0 */
        0xC8, /* COM 输出方向（从上到下）*/
        0x00, 0x10, /* 列地址低/高 */
        0x40, /* 显示起始行=0 */
        0x81, 0x7F, /* 对比度 */
        0xA1, /* 段重映射 */
        0xA6, /* 正常显示（不反色）*/
        0xA8, 0x3F, /* 多路比 = 64 */
        0xA4, /* 显示开启（RAM 内容）*/
        0xD3, 0x00, /* 显示偏移=0 */
        0xD5, 0x80, /* 时钟分频 */
        0xD9, 0xF1, /* 预充电周期 */
        0xDA, 0x12, /* COM 引脚硬件配置 */
        0xDB, 0x40, /* VCOMH 电平 */
        0x8D, 0x14, /* 电荷泵使能 */
        0xAF  /* 开显示 */
    };
    for (uint16_t i = 0; i < sizeof(init_cmds); i++) {
        OLED_WriteCmd(init_cmds[i]);
    }
    OLED_Clear();
    OLED_Display();
}

void OLED_Clear(void)
{
    for (uint16_t i = 0; i < sizeof(framebuffer); i++) {
        framebuffer[i] = 0;
    }
}

void OLED_Display(void)
{
    /* 分 8 页推屏，每页一个 I2C 事务（短，方便以后加锁时分小块）*/
    for (uint8_t page = 0; page < OLED_HEIGHT / 8; page++) {
        OLED_DisplayPage(page);
    }
}

void OLED_DisplayPage(uint8_t page)
{
    if (page >= OLED_HEIGHT / 8) return;

    OLED_WriteCmd(0xB0 + page);   /* 页地址 */
    OLED_WriteCmd(0x00);          /* 列低 */
    OLED_WriteCmd(0x10);          /* 列高 */
    OLED_WriteData(&framebuffer[page * OLED_WIDTH], OLED_WIDTH);
}

void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint16_t idx = (uint16_t)(y / 8) * OLED_WIDTH + x;
    if (on) {
        framebuffer[idx] |= (uint8_t)(1 << (y % 8));
    } else {
        framebuffer[idx] &= (uint8_t)~(1 << (y % 8));
    }
}

void OLED_DrawChar(uint8_t x, uint8_t y, char c)
{
    if (x > OLED_WIDTH - 6 || y > OLED_HEIGHT - 8) return;
    const uint8_t *glyph = font_6x8[(uint8_t)c - 32];
    for (uint8_t i = 0; i < 6; i++) {           /* 6 列 */
        for (uint8_t j = 0; j < 8; j++) {       /* 8 行 */
            if (glyph[i] & (1 << j)) {
                OLED_DrawPixel(x + i, y + j, 1);
            } else {
                OLED_DrawPixel(x + i, y + j, 0);
            }
        }
    }
}

void OLED_DrawString(uint8_t x, uint8_t y, const char *str)
{
    while (*str) {
        OLED_DrawChar(x, y, *str);
        x += 6;
        if (x > OLED_WIDTH - 6) break;
        str++;
    }
}
