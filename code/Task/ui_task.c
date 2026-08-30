#include "ui_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"          /* OLED_DrawString / OLED_Display */
#include "bsp_adc.h"       /* ADC_GetBatteryVoltage */
#include "control.h"       /* g_speed_aim/g_turn_aim + control_get_* */
#include "filter.h"        /* g_angle */
#include "mpu6050.h"       /* g_mpu_ok */
#include "remote_task.h"   /* remote_get_source */

/* ============ 数值 → 字符串（dec 位小数，0~3），不依赖 sprintf ============ */
static void fmt_num(char *buf, float val, uint8_t dec)
{
    char tmp[14];
    uint8_t n = 0, i;
    uint32_t whole, frac = 0, p = 1;
    uint8_t neg = 0;

    if (val < 0) { neg = 1; val = -val; }

    whole = (uint32_t)val;
    if (dec) {
        float f = val - (float)whole;
        for (i = 0; i < dec; i++) f *= 10.0f;
        frac = (uint32_t)(f + 0.5f);            /* 四舍五入 */
        for (i = 0; i < dec; i++) p *= 10;
        if (frac >= p) { frac = 0; whole++; }   /* 进位 */
    }

    /* 整数部分反序存入 tmp */
    do { tmp[n++] = (char)('0' + whole % 10); whole /= 10; } while (whole);
    if (neg) tmp[n++] = '-';

    /* 反转输出整数部分 */
    for (i = n; i > 0; i--) *buf++ = tmp[i - 1];

    /* 小数部分 */
    if (dec) {
        *buf++ = '.';
        for (i = 0; i < dec; i++) {
            p /= 10;
            *buf++ = (char)('0' + (frac / p) % 10);
        }
    }
    *buf = '\0';
}

void uiTask(void *argument)
{
    (void)argument;
    char buf[12];
    uint8_t src;

    OLED_Init();

    while (1) {
        vTaskDelay(250);                        /* 250ms 刷一页，8页共2秒刷完，分散 I2C 占用 */

        OLED_Clear();

        /* 第0行：俯仰角 + 当前速度 */
        OLED_DrawString(0, 0, "A");
        if (!g_mpu_ok) {
            OLED_DrawString(12, 0, "ERR");      /* MPU 未就绪 */
        } else {
            fmt_num(buf, g_angle, 1);
            OLED_DrawString(12, 0, buf);
        }
        OLED_DrawString(66, 0, "S");
        fmt_num(buf, control_get_speed(), 0);
        OLED_DrawString(78, 0, buf);

        /* 第1行：电池电压 + 当前控制源 */
        OLED_DrawString(0, 8, "B");

        fmt_num(buf, ADC_GetBatteryVoltage(), 1);
        OLED_DrawString(12, 8, buf);
        OLED_DrawString(66, 8, "R");
        src = remote_get_source();
        OLED_DrawString(78, 8, src == 1 ? "IR" : (src == 2 ? "WF" : "--"));
    

        /* 第2行：角度环 PID（P、D）*/
        OLED_DrawString(0, 16, "AP");
        fmt_num(buf, control_get_angle_kp(), 1);
        OLED_DrawString(24, 16, buf);
        OLED_DrawString(66, 16, "AD");
        fmt_num(buf, control_get_angle_kd(), 2);
        OLED_DrawString(90, 16, buf);

        /* 第3行：速度环 PID（P、I）*/
        OLED_DrawString(0, 24, "SP");
        fmt_num(buf, control_get_speed_kp(), 1);
        OLED_DrawString(24, 24, buf);
        OLED_DrawString(66, 24, "SI");
        fmt_num(buf, control_get_speed_ki(), 3);
        OLED_DrawString(90, 24, buf);

        /* 第4行：转向环 PID（P、D）*/
        OLED_DrawString(0, 32, "TP");
        fmt_num(buf, control_get_turn_kp(), 1);
        OLED_DrawString(24, 32, buf);
        OLED_DrawString(66, 32, "TD");
        fmt_num(buf, control_get_turn_kd(), 1);
        OLED_DrawString(90, 32, buf);

        /* 第5行：目标速度 + 目标转向 */
        OLED_DrawString(0, 40, "SA");
        fmt_num(buf, g_speed_aim, 0);
        OLED_DrawString(24, 40, buf);
        OLED_DrawString(66, 40, "TA");
        fmt_num(buf, g_turn_aim, 0);
        OLED_DrawString(90, 40, buf);
  

        /* 每次只刷一页，把原本集中的 8 页刷屏分散到 8 个周期 */
        static uint8_t display_page = 0;
        OLED_DisplayPage(display_page);
        display_page = (display_page + 1) % (OLED_HEIGHT / 8);
    }
}
