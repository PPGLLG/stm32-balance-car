#ifndef _BSP_ADC_H
#define _BSP_ADC_H

/* ============ 电池电压检测 ============
 * 12V 电池 → 电阻分压 → PA7 (ADC1_IN7)
 * 读取原始值 → 换算成 PA7 电压 → 乘分压倍数 → 得到电池电压
 *
 * CubeMX 已配置 ADC1_IN7（软件触发、单次转换），本驱动只是封装
 */

float ADC_GetBatteryVoltage(void); /* 返回电池电压（伏）*/

#endif /* _BSP_ADC_H */
