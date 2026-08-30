#include "ir_nec.h"
#include "tim.h"            /* htim3 */

/* ============ NEC 时间阈值（单位 µs，基于 TIM3 1µs/格）============
 * NEC 帧结构（接收头输出，高=空闲，低=收到38kHz）：
 *   ┌ 9ms 低(AGC) ─ 4.5ms 高 ─┐ 然后 32 位：
 *   每一位 = 560µs 低 + (560µs 高 = 0 | 1690µs 高 = 1)
 *
 * 我们用"下降沿间隔"解码：相邻两个下降沿之间 = 前一位的低 + 它的高。
 *   帧头间隔   ≈ 9ms + 4.5ms = 13.5ms
 *   位0 间隔   ≈ 560 + 560   = 1.12ms
 *   位1 间隔   ≈ 560 + 1690  = 2.25ms
 */
#define IR_HEADER_US         7000u   /* 间隔 ≥7ms 视为帧头（实际13.5ms）*/
#define IR_BIT_THRESHOLD     1600u   /* 间隔 <1.6ms=0，≥1.6ms=1 */
#define IR_MAX_BITS          32u     /* NEC 一帧 32 位 */
#define IR_REPEAT_WINDOW_MS  250u    /* 距离上次有效键码 250ms 内的长间隔才视为重复帧 */

/* ---- ISR 内部状态（只在 EdgeHandler 里用，不用 volatile）---- */
static uint16_t ir_last_cnt;        /* 上次边沿的计数器值 */
static uint8_t  ir_prev_valid;      /* 是否已有上次边沿（第一个沿跳过）*/
static uint8_t  ir_in_frame;        /* 是否正在接收一帧 */
static uint32_t ir_code;            /* 拼接中的 32 位码 */
static uint8_t  ir_bit_count;       /* 已拼位数（数到32）*/
static uint8_t  ir_last_key;        /* 上一次成功解码的键码（重复帧用）*/
static uint8_t  ir_have_last_key;   /* 是否已经有过一次成功键码 */
static uint32_t ir_last_key_ms;     /* 上一次成功键码的时间戳（ms） */

/* ---- ISR 与任务共享（必须 volatile）---- */
static volatile uint8_t ir_key_ready;   /* 键码就绪标志 */
static volatile uint8_t ir_keycode;     /* 解码出的命令键码 */

void IR_NEC_Init(void)
{
    ir_last_cnt   = 0;
    ir_prev_valid = 0;
    ir_in_frame   = 0;
    ir_code       = 0;
    ir_bit_count  = 0;
    ir_last_key   = 0;
    ir_have_last_key = 0;
    ir_last_key_ms = 0;
    ir_key_ready  = 0;
    ir_keycode    = 0;

    /* 启动 1µs 计数器，供量脉宽。定时器一旦跑起来就别停。 */
    HAL_TIM_Base_Start(&htim3);
}

void IR_NEC_EdgeHandler(void)
{
    uint16_t now = (uint16_t)htim3.Instance->CNT;   /* 当前计数 */
    uint16_t interval;

    /* 第一个边沿：还没有"上次"，只记录起点，不算间隔。
     * 这样每帧的 AGC 起点作为"第一个沿"，13.5ms 帧头间隔留给下一个沿。 */
    if (!ir_prev_valid)
    {
        ir_last_cnt = now;
        ir_prev_valid = 1;
        return;
    }

    /* 间隔 = 本次 - 上次。无符号减法自动处理计数器回绕，不用管。 */
    interval = (uint16_t)(now - ir_last_cnt);
    ir_last_cnt = now;

    /* 帧头：间隔很长（~13.5ms）→ 开始新一帧，清零重数 */
    if (interval >= IR_HEADER_US)
    {
        /* NEC 重复帧：上一次“帧头”之后没有任何数据位，并且之前有过有效键码，
         * 并且距离上次有效键码不超过 250ms，才判定为重复帧。
         * 超过时间窗则认为是新按键，开始正常解析新帧。 */
        if (ir_in_frame && ir_bit_count == 0 && ir_have_last_key &&
            (HAL_GetTick() - ir_last_key_ms) <= IR_REPEAT_WINDOW_MS)
        {
            ir_keycode     = ir_last_key;
            ir_key_ready   = 1;
            ir_last_key_ms = HAL_GetTick();  /* 刷新时间戳，保证后续重复帧也在窗口内 */
            ir_in_frame    = 0;
            ir_prev_valid  = 0;   /* 让下一个重复帧重新从 AGC 起点开始 */
            return;
        }

        ir_code      = 0;
        ir_bit_count = 0;
        ir_in_frame  = 1;
        return;
    }

    /* 不在帧内，忽略（噪声/空闲） */
    if (!ir_in_frame) return;

    /* 解码一位：按间隔长短判断 0 还是 1，拼进 32 位变量 */
    ir_code = (ir_code << 1) | ((interval >= IR_BIT_THRESHOLD) ? 1u : 0u);
    ir_bit_count++;

    if (ir_bit_count >= IR_MAX_BITS)     /* 凑满 32 位，一帧完成 */
    {
        /* NEC 32 位 = 16地址 + 8命令 + 8反码，校验命令和反码互补 */
        uint8_t cmd = (uint8_t)(ir_code >> 8);
        uint8_t inv = (uint8_t)(ir_code & 0xFF);
        uint8_t inv_check = (uint8_t)(~inv);
        if (cmd == inv_check)
        {
            ir_last_key      = cmd;
            ir_have_last_key = 1;
            ir_last_key_ms   = HAL_GetTick();
            ir_keycode       = cmd;
            ir_key_ready     = 1;
        }
        ir_in_frame = 0;
        ir_prev_valid = 0;     /* 让下一帧的 AGC 起点重新作为"第一个沿" */
    }
}

uint8_t IR_NEC_KeyReady(void)
{
    return ir_key_ready;
}

uint8_t IR_NEC_GetKey(void)
{
    uint8_t key = ir_keycode;
    ir_keycode   = 0;
    ir_key_ready = 0;
    return key;
}
