#include "soft_i2c.h"
#include "main.h"        /* SCL_Pin/SDA_Pin（PB8/PB9）*/

/* ============ 软件模拟 I2C 实现 ============
 * 开漏输出：写 1 = 释放引脚（靠上拉拉高），可读从机 ACK。
 * 慢速（~40kHz）+ 慢翻转（SPEED_LOW）：波形干净，抗杜邦线毛刺。
 */

static void bb_delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 10; i++);   /* 提速：原来40，I2C约4倍快，MPU读3ms→约1ms，降低控制延迟 */
}

#define BB_SCL(v)  HAL_GPIO_WritePin(GPIOB, SCL_Pin, (v) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define BB_SDA(v)  HAL_GPIO_WritePin(GPIOB, SDA_Pin, (v) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* 起始：SCL 高时 SDA 拉低 */
static void bb_start(void)
{
    BB_SDA(1); BB_SCL(1); bb_delay();
    BB_SDA(0); bb_delay();
    BB_SCL(0); bb_delay();
}

/* 停止：SCL 高时 SDA 拉高 */
static void bb_stop(void)
{
    BB_SDA(0); bb_delay();
    BB_SCL(1); bb_delay();
    BB_SDA(1); bb_delay();
}

/* 发一位：数据先就绪 → SCL 拉高采样 → 拉低 */
static void bb_tx_bit(uint8_t v)
{
    BB_SDA(v); bb_delay();
    BB_SCL(1); bb_delay();
    BB_SCL(0); bb_delay();
}

/* 发一个字节，返回 ACK 位：0=有应答，非0=无应答 */
static uint8_t bb_tx_byte(uint8_t byte)
{
    uint8_t i, ack;

    for (i = 8; i > 0; i--) {
        bb_tx_bit((byte >> (i - 1)) & 0x01);
    }
    BB_SDA(1); bb_delay();               /* 释放 SDA，读应答 */
    BB_SCL(1); bb_delay();
    ack = HAL_GPIO_ReadPin(GPIOB, SDA_Pin);
    BB_SCL(0); bb_delay();
    return ack;
}

/* 读一个字节（调用前需释放 SDA，由从机驱动）*/
static uint8_t bb_rx_byte(void)
{
    uint8_t i, byte = 0;

    BB_SDA(1);                           /* 释放 SDA */
    for (i = 8; i > 0; i--) {
        BB_SCL(1); bb_delay();
        byte = (uint8_t)((byte << 1) | HAL_GPIO_ReadPin(GPIOB, SDA_Pin));
        BB_SCL(0); bb_delay();
    }
    return byte;
}

void soft_i2c_init(void)
{
    GPIO_InitTypeDef g = {0};

    /* 解除 HAL I2C1 的 AF_OD 复用，配成普通开漏输出 */
    HAL_GPIO_DeInit(GPIOB, SCL_Pin | SDA_Pin);
    g.Pin   = SCL_Pin | SDA_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, SCL_Pin | SDA_Pin, GPIO_PIN_SET);  /* 空闲拉高 */
}

/* addr8 用 8 位地址（如 0x78 = 0x3C<<1），发 地址+数据，返回 0=全应答 */
uint8_t soft_i2c_write(uint8_t addr8, const uint8_t *data, uint16_t len)
{
    uint8_t ack;

    bb_start();
    ack = bb_tx_byte(addr8);             /* 写方向（bit0=0）*/
    if (ack) { bb_stop(); return ack; }
    while (len--) {
        ack = bb_tx_byte(*data++);
        if (ack) break;
    }
    bb_stop();
    return ack;
}

/* 读 n 字节：返回 0=成功，非0=地址无应答 */
uint8_t soft_i2c_read(uint8_t addr8, uint8_t *buf, uint16_t len)
{
    uint8_t ack;

    bb_start();
    ack = bb_tx_byte((uint8_t)(addr8 | 0x01));   /* 读方向（bit0=1）*/
    if (ack) { bb_stop(); return ack; }
    while (len--) {
        *buf++ = bb_rx_byte();
        /* 非最后字节：ACK（拉低）；最后字节：NACK（释放）*/
        BB_SDA((len == 0) ? 1 : 0);
        bb_delay();
        BB_SCL(1); bb_delay();
        BB_SCL(0); bb_delay();
    }
    bb_stop();
    return 0;
}
