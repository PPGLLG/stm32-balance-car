#include "bsp_adc.h"
#include "adc.h"        /* hadc1（CubeMX 已初始化 ADC1_IN7）*/

/* 12 位 ADC 换算 */
#define ADC_REF_VOLTAGE   3.3f     /* 参考电压（V）*/
#define ADC_MAX_VALUE     4095.0f  /* 12位最大值 */

/* ⚠️ 分压倍数 = 电池电压 / PA7电压，按你实际电阻算！
 * 例：R1(电池侧) + R2(地侧)，PA7 接在 R2 上，倍数 = (R1+R2)/R2。
 * 默认 4.0 只是占位，上板实测校准。*/
#define ADC_DIVIDER_RATIO 4.0f

float ADC_GetBatteryVoltage(void)
{
    uint32_t raw = 0;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    /* PA7电压 = raw/4095×3.3 → 电池电压 = ×分压倍数 */
    return (float)raw / ADC_MAX_VALUE * ADC_REF_VOLTAGE * ADC_DIVIDER_RATIO;
}
