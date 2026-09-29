#include "pid.h"
#include <math.h>

/* =================================================================
 * 串级 PID (位置环 + 速度环)
 * ================================================================= */
void DualPID_Init(DualPID_Controller *pid,
                  float pos_Kp, float pos_Ki, float pos_Kd,
                  float vel_Kp, float vel_Ki, float vel_Kd,
                  float max_vel, float max_output,
                  float vel_ff, float vel_ff_dead)
{
    pid->pos_Kp = pos_Kp;
    pid->pos_Ki = pos_Ki;
    pid->pos_Kd = pos_Kd;
    pid->vel_Kp = vel_Kp;
    pid->vel_Ki = vel_Ki;
    pid->vel_Kd = vel_Kd;
    pid->max_vel = max_vel;
    pid->max_output = max_output;
    pid->vel_ff = vel_ff;
    pid->vel_ff_dead = vel_ff_dead;
    pid->pos_integral = 0;
    pid->pos_error_prev = 0;
    pid->vel_integral = 0;
    pid->vel_error_prev = 0;
}

void DualPID_SetMaxVel(DualPID_Controller *pid, float max_vel)
{
    pid->max_vel = max_vel;
}

float DualPID_Update(DualPID_Controller *pid,
                     float target_pos, float current_pos,
                     float current_vel, float dt, float vel_bias)
{
    /* ---------- 位置环 ---------- */
    float pos_error = target_pos - current_pos;

    /* ⭐ 位置环积分: 只在"接近目标"(|误差| < POS_INTEGRAL_BAND)时才累积 (2026-09-27 改)
     * 为什么加: pos_Ki=0(纯 P)时, 被静摩擦卡住的轮子残差 = 克服静摩擦所需PWM/pos_Kp,
     *          实测 20~50 计数, 而且四轮不等 → 四轮在不同位置停住 → 停车瞬间车被拧一下、
     *          每段车姿都偏一点, 几十段累积成"越走越偏/停不回原位"。
     *          加上积分后, 卡住的轮子会持续加大 PWM 直到磨到位(残差 → 几个计数)。
     * 为什么加带宽: 长距离段内不累积 → 不会积分饱和导致到点猛冲/超调;
     *          只在最后约 18mm 内起作用, 对"走直/走快"没有任何影响。
     * 上限 max_vel*0.23 → 最大超调 ≈ 上限/pos_Kp (十几计数, 可接受)。
     * 位置环积分限幅 = 期望速度上限的 23% (经验系数, 防积分饱和导致到点猛冲) */
    const float POS_INTEGRAL_BAND = 120.0f;   /* 计数 ≈ 18mm */
    if (fabsf(pos_error) < POS_INTEGRAL_BAND)
    {
        pid->pos_integral += pid->pos_Ki * pos_error * dt;
        float integral_limit_pos = pid->max_vel * 0.23f;
        if (pid->pos_integral > integral_limit_pos)  pid->pos_integral = integral_limit_pos;
        if (pid->pos_integral < -integral_limit_pos) pid->pos_integral = -integral_limit_pos;
    }

    float pos_D = pid->pos_Kd * (pos_error - pid->pos_error_prev) / dt;

    float vel_target = pid->pos_Kp * pos_error + pid->pos_integral + pos_D;

    /* ---------- 航向保持速度偏置 (2026-09-27 重整) ----------
     * 1) 偏置自身限幅(≤0.7×max_vel): 防止"偏置比平移速度还大"时把车拧成原地打转;
     * 2) 不允许偏置把"平移方向"反掉(最多把某轮减到 0): 实测踩过——位置环顶在
     *    max_vel 时, ±25 的偏置会让一侧轮子反转, 另一侧正转 → 车原地打转卡死
     *    (10s 超时兜底后 e: 残留 57 计数, 切走状态后车又猛拱一下);
     * 3) 位置环输出按 (max_vel - |偏置|) 限幅 → 给偏置留出余量。否则位置环一旦
     *    饱和(平移途中基本都顶在 max_vel), 加在"减速侧"的偏置生效、加在"加速侧"
     *    的被钳掉 → 差速不对称, 航向修正实际只发挥了一半。 */
    float bias = vel_bias;
    float bias_max = pid->max_vel * 0.7f;
    if (bias >  bias_max) bias =  bias_max;
    if (bias < -bias_max) bias = -bias_max;

    float pos_lim = pid->max_vel - fabsf(bias);
    if (pos_lim < 0.0f) pos_lim = 0.0f;
    vel_target = fmaxf(fminf(vel_target, pos_lim), -pos_lim);

    /* 不允许偏置反向: 正向平移时偏置最多把该轮减到 0(反向平移同理) */
    if (vel_target > 0.0f && bias < -vel_target) bias = -vel_target;
    if (vel_target < 0.0f && bias > -vel_target) bias = -vel_target;

    vel_target += bias;
    vel_target = fmaxf(fminf(vel_target, pid->max_vel), -pid->max_vel);

    pid->pos_error_prev = pos_error;

    /* ---------- 速度环 ---------- */
    float vel_error = vel_target - current_vel;

    /* 速度环积分限幅 = PWM 上限的 23% (经验系数, 防积分饱和导致 PWM 顶满不回落)
     * 一般无需调整 */
    pid->vel_integral += pid->vel_Ki * vel_error * dt;
    float integral_limit_vel = pid->max_output * 0.23f;
    if (pid->vel_integral > integral_limit_vel)  pid->vel_integral = integral_limit_vel;
    if (pid->vel_integral < -integral_limit_vel) pid->vel_integral = -integral_limit_vel;

    float vel_D = pid->vel_Kd * (vel_error - pid->vel_error_prev) / dt;

    /* 前馈: 线性速度->PWM + 平滑死区补偿 (死区项在速度过零附近线性过渡, 避免阶跃抖振)
     * 阈值 8.0f = '认为接近零速'的速度范围(计数/周期), 一般无需调整;
     * 死区补偿的强度由 pid->vel_ff_dead 控制(见 pid.h 说明, 在 Chassis.c 里设初值) */
    float ff = pid->vel_ff * vel_target;                            //速度前馈项
    if (vel_target > 8.0f)       ff += pid->vel_ff_dead;            //速度死区补偿
    else if (vel_target < -8.0f) ff -= pid->vel_ff_dead;
    else                         ff += pid->vel_ff_dead * (vel_target / 8.0f);//速度死区补偿线性过渡(vel_target / 8.0f)

    float output = pid->vel_Kp * vel_error + pid->vel_integral + vel_D + ff;
    output = fmaxf(fminf(output, pid->max_output), -pid->max_output);

    pid->vel_error_prev = vel_error;

    return output;
}

/* =================================================================
 * 单环位置式 PID (转向环)
 * ================================================================= */
void pid_param_init(pid_param_t *pid, float kp, float ki, float kd, float imax)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->imax = imax;
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->out_p = 0.0f;
    pid->out_i = 0.0f;
    pid->out_d = 0.0f;
    pid->out = 0.0f;
}
/* =================================================================
 * 位置式 PID 控制器 (单环)
 * ================================================================= */ 
float PidLocCtrl(pid_param_t *pid, float error)
{
    pid->integrator += error;                                       //误差累加
    if (pid->integrator > pid->imax)  pid->integrator = pid->imax;  //积分限幅
    if (pid->integrator < -pid->imax) pid->integrator = -pid->imax;

    pid->out_p = pid->kp * error;                                   //比例项p
    pid->out_i = pid->ki * pid->integrator;                         //积分项i
    pid->out_d = pid->kd * (error - pid->last_error);               //微分项d

    pid->last_error = error;                                        //保存本次误差, 供下次计算微分项使用

    pid->out = pid->out_p + pid->out_i + pid->out_d;                //总输出
    return pid->out;
}

/* =================================================================
 * 复位函数
 * ================================================================= */
void DualPID_Reset(DualPID_Controller *pid)
{
    pid->pos_integral = 0;
    pid->pos_error_prev = 0;
    pid->vel_integral = 0;
    pid->vel_error_prev = 0;
}

void pid_param_reset(pid_param_t *pid)
{
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->out_p = 0.0f;
    pid->out_i = 0.0f;
    pid->out_d = 0.0f;
    pid->out = 0.0f;
}
