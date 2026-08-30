#include <stdint.h>       /* uint8_t / uint16_t（跌倒锁存）*/
#include "control.h"
#include "filter.h"       /* g_angle（互补滤波后的俯仰角）*/
#include "mpu6050.h"      /* g_mpu + GYRO_SENSITIVITY */
#include "bsp_motor.h"    /* setPWM1/2, read_encoder1/2 */

/* ============ 目标值（遥控通过邮箱写入最终目标，由斜坡逼近）============ */
float g_speed_aim = 0.0f;    /* 当前斜坡后的目标速度：0 = 原地站 */
float g_speed_target = 0.0f; /* 期望最终目标速度：遥控写入，由斜坡逐步逼近 */
float g_turn_aim  = 0.0f;    /* 当前斜坡后的目标转向：0 = 不转 */
float g_turn_target = 0.0f;  /* 期望最终目标转向：遥控写入，由斜坡逐步逼近 */
float g_angle_aim = 3.0f;    /* 平衡角（度）：实测机械平衡点约 3°，可用 WiFi A 命令微调 */
float g_yaw_rate_now = 0.0f; /* 实际偏航角速度（°/s），供调试显示 */

/* ============ PID 参数与状态（static = 只有本文件用）============ */
static float angle_kp, angle_kd;      /* 角度环 PD */
static float speed_kp, speed_ki;      /* 速度环 PI */
static float speed_integral;          /* 速度积分累积 */
static float turn_kp,  turn_kd;       /* 转向环 PD */

/* 环之间的中间量 */
static float common_pwm;   /* 共同 PWM（±1）*/
static float diff_pwm;     /* 差速 PWM（±1）*/
static float g_speed_now;  /* 当前实际速度（显示任务经 getter 读）*/
static float speed_lpf;    /* 编码器速度一阶低通 */
static float speed_out;    /* 速度环输出（调参/波形用）*/
static float gyro_lpf;     /* 陀螺俯仰角速度一阶低通 */
static uint8_t  fallen;          /* 跌倒锁存：1=已倒，扶正后才解锁 */
static uint16_t upright_cycles;  /* 扶正持续计数 */

/* 可调常数 */
#define SPEED_ERR_INTEGRATE_LIMIT  500.0f  /* 积分分离阈值：放宽，让加速过程中积分也能累积 */
#define SPEED_INTEGRAL_MAX        20000.0f /* 速度积分限幅：加大，避免提前饱和 */
#define SPEED_RAMP_STEP           2.0f     /* 目标速度斜坡步长：每个速度外环周期增加/减少 2 */
#define TURN_RAMP_STEP            1.0f     /* 目标转向斜坡步长：每个控制周期增加/减少 1 */
#define PWM_DEAD_ZONE             0.01f    /* 输出死区补偿：实测约 1% 占空比 */
#define FALL_ANGLE               60.0f    /* 跌倒保护：超过该角度认为已跌倒，停止输出并锁存 */
#define RELEASE_ANGLE            30.0f    /* 扶正到 ±30° 内即可解锁（放宽，避免之前锁死）*/
#define RELEASE_HOLD_CYCLES      3        /* 稳定 30ms 就解锁（之前200ms太苛刻，车扶起也不解锁）*/

void control_init(void)
{
    /* 完整直立环起步：角度PD + 速度PI（参数经 WiFi/代码两处可调）*/
    angle_kp = -0.0048f;     /* 角度环 P */
    angle_kd =  0.000024f;   /* 角度环 D */
    speed_kp =  1.5f;     /* 速度环 P */
    speed_ki =  0.025f;   /* 速度环 I（积分分离只在低速时累积）*/
    turn_kp  =  0.005f;     /* 转向环 P：前馈无反馈版本 */
    turn_kd  =  0.0f;       /* 转向环 D：当前未使用 */

    speed_integral = 0.0f;
    speed_lpf = 0.0f;
    speed_out = 0.0f;
    g_speed_aim = 0.0f;
    g_speed_target = 0.0f;
    g_turn_aim = 0.0f;
    g_turn_target = 0.0f;
    gyro_lpf  = 0.0f;
    common_pwm = 0.0f;
    diff_pwm   = 0.0f;
    fallen = 0;
    upright_cycles = 0;
}

/* ============ 速度环（外环）：低速更新，只算角度修正量 ============ */
void control_speed_update(void)
{
    /* ① 实际速度 = 两轮编码器之和（= 平均速度的倍数）*/
    /*    实测：车前进时编码器读数为负 → 整体取反，使"前进 = 正速度" */
    float speed_raw = -(float)(read_encoder1() + read_encoder2());

    /* 一阶低通：滤掉编码器计数抖动，速度环更稳 */
    speed_lpf += 0.2f * (speed_raw - speed_lpf);
    g_speed_now = speed_lpf;   /* 显示/波形用滤波后的速度 */

    /* ② 目标速度斜坡：避免阶跃导致车猛冲/猛刹 */
    if (g_speed_aim < g_speed_target) {
        g_speed_aim += SPEED_RAMP_STEP;
        if (g_speed_aim > g_speed_target) g_speed_aim = g_speed_target;
    } else if (g_speed_aim > g_speed_target) {
        g_speed_aim -= SPEED_RAMP_STEP;
        if (g_speed_aim < g_speed_target) g_speed_aim = g_speed_target;
    }

    /* ③ 速度环 PI（外环）→ 输出"角度修正" */
    float speed_bias = g_speed_aim - speed_lpf;

    /* 积分分离：速度误差大时不积分（防超调），误差小才积分（消静差）*/
    if (speed_bias > -SPEED_ERR_INTEGRATE_LIMIT &&
        speed_bias <  SPEED_ERR_INTEGRATE_LIMIT)
    {
        speed_integral += speed_bias;
        /* 积分限幅：防积分饱和 */
        if (speed_integral >  SPEED_INTEGRAL_MAX) speed_integral =  SPEED_INTEGRAL_MAX;
        if (speed_integral < -SPEED_INTEGRAL_MAX) speed_integral = -SPEED_INTEGRAL_MAX;
    }

    speed_out = speed_bias * speed_kp + speed_integral * speed_ki;
}

/* ============ 角度环（内环）：每个控制周期都更新，直接算共同 PWM ============ */
void control_angle_update(void)
{
    /* 外环修正叠进角度目标：速度太快 → 修正为负 → 目标角度变小 → 往后仰 → 减速 */
    float angle_bias = g_angle_aim + speed_out - g_angle;
    //目标值angle_bias = 平衡角 + 速度环修正 - 实际角度
    /* 微分先行：D 用陀螺仪俯仰角速度（不是误差微分）
     * 注意符号与 filter.c 保持一致：负号使陀螺方向与角度方向一致 */
    float gyro_pitch = -((float)g_mpu.gyro[1] - g_gyro_offset[1]) / GYRO_SENSITIVITY;

    /* 一阶低通：滤陀螺高频噪声，ad 才能加大提供换向提前量而不抖 */
    gyro_lpf += 0.05f * (gyro_pitch - gyro_lpf);

    common_pwm = angle_bias * angle_kp + gyro_lpf * angle_kd;
}

/* ============ 转向（并列环）：目标偏航角速度 → 差速 ============ */
void control_turn(void)
{
    /* 目标转向斜坡：避免转向目标阶跃导致突然猛转 */
    if (g_turn_aim < g_turn_target) {
        g_turn_aim += TURN_RAMP_STEP;
        if (g_turn_aim > g_turn_target) g_turn_aim = g_turn_target;
    } else if (g_turn_aim > g_turn_target) {
        g_turn_aim -= TURN_RAMP_STEP;
        if (g_turn_aim < g_turn_target) g_turn_aim = g_turn_target;
    }

    /* 没有转向指令时，强制关闭转向输出，防止陀螺零偏/噪声导致原地自转 */
    if (g_turn_aim > -0.01f && g_turn_aim < 0.01f) {
        diff_pwm = 0.0f;
        return;
    }

    /* 只记录实际偏航角速度供调试显示，不参与控制 */
    g_yaw_rate_now = ((float)g_mpu.gyro[2] - g_gyro_offset[2]) / GYRO_SENSITIVITY;

    /* 无反馈前馈式转向：只按目标转向输出差速。
     * 当前硬件方向已确认，整体取负以匹配之前的转向极性。 */
    diff_pwm = -g_turn_aim * turn_kp;
}

/* ============ 输出：合成 + 死区 + 电机 ============ */
void control_output(void)
{
    /* ① MPU 初始化失败 → 不驱动电机（防拿脏数据乱动）*/
    if (!g_mpu_ok) {
        setPWM1(0);
        setPWM2(0);
        return;
    }

    /* ② 跌倒锁存：超限锁死；扶正到 RELEASE_ANGLE 内并稳定一会才解锁 */
    if (g_angle > FALL_ANGLE || g_angle < -FALL_ANGLE) {
        fallen = 1;
        upright_cycles = 0;
    } else if (g_angle < RELEASE_ANGLE && g_angle > -RELEASE_ANGLE) {
        if (++upright_cycles >= RELEASE_HOLD_CYCLES) {
            fallen = 0;
            upright_cycles = 0;
        }
    } else {
        upright_cycles = 0;
    }

    if (fallen) {
        speed_integral = 0;      /* 清积分防恢复后爆冲 */
        speed_lpf = 0.0f;        /* 清速度滤波值，防恢复后旧速度拖住控制 */
        setPWM1(0);
        setPWM2(0);
        return;
    }

    /* ① 合成：左轮 = 共同 - 差速，右轮 = 共同 + 差速（转向极性可实测交换）*/
    /* ★ PWM 限幅：输出上限 0.8，给足加速/回正力矩，同时保留一定余量避免满占空比 */
    if (common_pwm >   0.8f) common_pwm =  0.8f;
    if (common_pwm <  -0.8f) common_pwm = -0.8f;

    float left  = common_pwm - diff_pwm;
    float right = common_pwm + diff_pwm;

    /* ② 输出偏移（死区补偿）：跳过电机静摩擦，让微小修正也能驱动。
     *   注意：零输出不加死区，否则平衡点会有一个恒定 ±0.03 的漂移输出，
     *   电机在车立正时仍会一直朝一个方向转。 */
    if (left  >  0.001f)      left  += PWM_DEAD_ZONE;
    else if (left < -0.001f)  left  -= PWM_DEAD_ZONE;
    else                      left  = 0.0f;
    if (right >  0.001f)      right += PWM_DEAD_ZONE;
    else if (right < -0.001f) right -= PWM_DEAD_ZONE;
    else                      right = 0.0f;

    /* ③ 输出（setPWM 内部会限幅 ±1）*/
    setPWM1(left);
    setPWM2(right);
}

/* ============ 只读 getter（显示任务用，只读不改）============ */
float control_get_angle_kp(void) { return angle_kp; }
float control_get_angle_kd(void) { return angle_kd; }
float control_get_speed_kp(void) { return speed_kp; }
float control_get_speed_ki(void) { return speed_ki; }
float control_get_turn_kp(void)  { return turn_kp; }
float control_get_turn_kd(void)  { return turn_kd; }
float control_get_speed(void)    { return g_speed_now; }
float control_get_speed_out(void) { return speed_out; }
float control_get_pwm(void)      { return common_pwm; }

/* ============ 参数 setter（WiFi 实时调参用，改参数不清积分）============ */
void control_set_angle_kp(float v) { angle_kp = v; }
void control_set_angle_kd(float v) { angle_kd = v; }
void control_set_speed_kp(float v) { speed_kp = v; }
void control_set_speed_ki(float v) { speed_ki = v; speed_integral = 0.0f; } /* 改 I 就清积分，防旧积分捣乱 */
void control_set_turn_kp(float v)  { turn_kp  = v; }
void control_set_turn_kd(float v)  { turn_kd  = v; }
