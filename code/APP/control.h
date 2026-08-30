#ifndef _CONTROL_H
#define _CONTROL_H

/* ============ 目标值 ============
 * 暂时用全局变量（速度/转向先=0），
 * 以后遥控任务写这几个变量（或改从邮箱读）*/
extern float g_speed_aim;   /* 当前斜坡后的目标速度：0=原地站，正=前进，负=后退 */
extern float g_speed_target; /* 期望最终目标速度：遥控写入，由斜坡逐步逼近 */
extern float g_turn_aim;    /* 当前斜坡后的目标偏航角速度：0=不转 */
extern float g_turn_target; /* 期望最终目标偏航角速度：遥控写入，由斜坡逐步逼近 */
extern float g_angle_aim;   /* 平衡角（度）：车立住的目标角度，一般固定 */
extern float g_yaw_rate_now; /* 实际偏航角速度（°/s），供调试显示 */

void control_init(void);           /* 开机调用一次：初始化 PID 参数 */
void control_speed_update(void);   /* 速度外环：低速更新，算角度修正量 */
void control_angle_update(void);   /* 角度内环：每个控制周期更新，算共同 PWM */
void control_turn(void);           /* 转向 → 算差速 PWM */
void control_output(void);         /* 合成 + 死区 + 输出电机 */

/* ============ 只读 getter（显示任务用）============ */
float control_get_angle_kp(void);
float control_get_angle_kd(void);
float control_get_speed_kp(void);
float control_get_speed_ki(void);
float control_get_turn_kp(void);
float control_get_turn_kd(void);
float control_get_speed(void);    /* 当前实际速度（编码器之和，控制环刚算的）*/
float control_get_speed_out(void); /* 速度环输出（调参/波形用）*/
float control_get_pwm(void);      /* 共同PWM（角度+速度环合成，Vofa+调参看）*/

/* ============ 参数 setter（WiFi 实时调参用）============ */
void control_set_angle_kp(float v);
void control_set_angle_kd(float v);
void control_set_speed_kp(float v);
void control_set_speed_ki(float v);
void control_set_turn_kp(float v);
void control_set_turn_kd(float v);

#endif
