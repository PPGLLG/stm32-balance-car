#include "test_task.h"
#include "FreeRTOS.h"     /* pdMS_TO_TICKS */
#include "task.h"         /* vTaskDelay */
#include "bsp_motor.h"    /* PWMinit / setPWM1/2 / read_encoder1/2 */
#include "translate.h"    /* g_vofa_line 波形共享缓冲（remote 任务经 WiFi 发 Vofa）*/

/* 整数 → 字符串（带符号），返回新的写入位置 */
static uint16_t fmt_int(char *buf, uint16_t pos, int32_t v)
{
    char tmp[12];
    uint8_t n = 0;
    uint32_t u = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    do { tmp[n++] = (char)('0' + u % 10); u /= 10; } while (u);
    if (v < 0) buf[pos++] = '-';
    while (n) buf[pos++] = tmp[--n];
    return pos;
}

/* ============ 带载死区扫描（车放地上！）============
 * 目的：测车落地时（轮子着地、带车重），电机多少占空比才开始推动车。
 * 行为：占空比 0% → 40%，每 1 秒 +1%，到 40% 归零重新扫。两轮同向。
 *       波形 CH0 = 当前占空比（%），CH1 = 编码器速度（每秒脉冲数）。
 * 判读：CH1（速度）从 0 开始变非 0 的那一拍，对应 CH0 = 带载死区。
 *       （空载死区是 3~4%，带载会大很多，这就是"倒60°电机才动"的原因）
 * ⚠️ 车放地上轮子着地，扫描中车会被推着走：
 *    - 用护栏/墙挡住车，或手扶车顶让它原地（别让轮子堵死，轻扶即可）；
 *    - 轮子被堵死 = 堵转，会烧板，务必让轮子能转。
 * 测试模式：freertos.c 里 TASK_CONTROL_ON=0、TASK_TEST_ON=1、TASK_REMOTE_ON=1 */
void testTask(void *argument)
{
    uint16_t percent = 0;   /* 当前占空比（%，0~40）*/

    (void)argument;
    PWMinit();              /* 启动 PWM 输出 + 编码器计数 */

    while (1) {
        float pwm = (float)percent * 0.01f;
        setPWM1(pwm);
        setPWM2(pwm);

        /* 读编码器（读后清零），两轮之和 = 每秒脉冲数 */
        int32_t speed = (int32_t)read_encoder1() + (int32_t)read_encoder2();

        /* 组波形行：占空比,速度\n（FireWater 逗号分隔）*/
        char line[32];
        uint16_t pos = 0;
        pos = fmt_int(line, pos, (int32_t)percent);
        line[pos++] = ',';
        pos = fmt_int(line, pos, speed);
        line[pos++] = '\n';

        for (uint16_t i = 0; i < pos; i++) g_vofa_line[i] = line[i];
        g_vofa_line_len = pos;
        g_vofa_ready = 1;              /* 通知 remote 任务转发给 Vofa */

        percent++;
        if (percent > 40) percent = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));   /* 1 秒一步 */
    }
}
