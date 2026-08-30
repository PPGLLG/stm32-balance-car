#ifndef _BSP_MOTOR_H          // ← 防止重复包含（必须）
#define _BSP_MOTOR_H

#include <stdint.h>          /* int16_t 等整数类型 */
//从PB12和PB13控制方向，通过定时器1，从PA8和PA11输出PWM波，控制电机的转速
    //注意：一个GPIO口通过反相器控制AIN1和AIN2，另一个GPIO口通过反相器控制BIN1和BIN2
    //因此AIN1必然和AIN2反相，BIN1必然和BIN2反相
//从PA0和PA1，PB6和PB7，通过定时器2,4读取编码器的计数值，计算电机的转速
void PWMinit(void);
void setPWM1(float v1);       /* 用于传输pwm波 */
void setPWM2(float v2);       /* 这里的1和2分别对应两个电机的PWM波，
                               * v1和v2分别对应两个电机的占空比，范围为-1~1，
                               * 正数表示正转，负数表示反转。
                               * 并且后续所有有关速度的变量都需要用范围为-1~1的带符号浮点数表示 */

int16_t read_encoder1(void);
int16_t read_encoder2(void);
///*用于读取编码器的计数值，返回的就是计数值，不是速度也不是占空比，
//这个函数用一次，就读一次，清一次计数值
//后续处理可以换算成速度或者占空比，正数表示正转，负数表示反转。*/

#endif // _BSP_MOTOR_H