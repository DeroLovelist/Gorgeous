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

    /* 位置环积分限幅 = 期望速度上限的 23% (经验系数, 防积分饱和导致到点猛冲)
     * 一般无需调整: 调大→积分消除误差能力更强但更易超调, 调小→更稳但偏慢 */
    pid->pos_integral += pid->pos_Ki * pos_error * dt;
    float integral_limit_pos = pid->max_vel * 0.23f;
    if (pid->pos_integral > integral_limit_pos)  pid->pos_integral = integral_limit_pos;
    if (pid->pos_integral < -integral_limit_pos) pid->pos_integral = -integral_limit_pos;

    float pos_D = pid->pos_Kd * (pos_error - pid->pos_error_prev) / dt;

    float vel_target = pid->pos_Kp * pos_error + pid->pos_integral + pos_D;
    vel_target = fmaxf(fminf(vel_target, pid->max_vel), -pid->max_vel);
    vel_target += vel_bias;   /* 航向保持偏置: 必须在限幅后叠加, 否则位置环饱和时被吞掉 */
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
