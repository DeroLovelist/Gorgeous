#include "pid.h"
#include "motor.h"    // 假设 motor.h 提供了 motor_set_pwm 函数声明
#include "encoder.h"  // 假设 encoder.h 提供了 get_encoder_data 函数声明
#include <math.h> 
#include <stdio.h> 

// =================================================================
// 1. 全局变量定义
// =================================================================
// 定义四个电机的串级PID控制器实例
DualPID_Controller pid_motor_a;
DualPID_Controller pid_motor_b;
DualPID_Controller pid_motor_c;
DualPID_Controller pid_motor_d;

// 定义转向环PID控制器实例
pid_param_t pid_steering;

// 当前电机状态变量 (由 get_encoder_data 更新)
int16_t current_speed_a, current_speed_b, current_speed_c, current_speed_d;
int32_t current_pos_a, current_pos_b, current_pos_c, current_pos_d;

// =================================================================
// 2. 初始化函数
// =================================================================
/**
 * @brief  初始化所有PID控制器 (电机串级PID + 转向环PID)
 */
void pid_init_all(void)
{
    // ---------- 1. 初始化电机串级PID ----------
    // 速度环参数 (使用您已调好的值)
    float final_vel_Kp = 0.08f;
    float final_vel_Ki = 0.2f;
    
    // 位置环参数 (使用您已调好的值)
   // float initial_pos_Kp = 0.03f;
	 //  float initial_pos_Kp = 0.3f;//比0.03停的更急
     float initial_pos_Kp = 0.6f;
    float initial_pos_Ki = 0.0f;
    float initial_pos_Kd = 0.0f;//调了会震荡
    
    // 调用初始化函数为每个电机配置参数
    DualPID_Init(&pid_motor_a, initial_pos_Kp, initial_pos_Ki, initial_pos_Kd, final_vel_Kp, final_vel_Ki, 0.0f, 1500, 199);
    DualPID_Init(&pid_motor_b, initial_pos_Kp, initial_pos_Ki, initial_pos_Kd, final_vel_Kp, final_vel_Ki, 0.0f, 1500, 199);
    DualPID_Init(&pid_motor_c, initial_pos_Kp, initial_pos_Ki, initial_pos_Kd, final_vel_Kp, final_vel_Ki, 0.0f, 1500, 199);
    DualPID_Init(&pid_motor_d, initial_pos_Kp, initial_pos_Ki, initial_pos_Kd, final_vel_Kp, final_vel_Ki, 0.0f, 1500, 199);
    
    // ---------- 2. 初始化转向环PID ----------
    // ★★★ 这些是转向控制的初始调试参数，需要您根据实际情况调整 ★★★
    float steering_kp = 3.9f;   // 响应速度
	 // float steering_kp = 5.5f;   // 响应速度

    float steering_ki = 0.0f;   // 静态误差消除 (先关闭)
    float steering_kd = 1.0f;   // 抑制震荡
    float steering_imax = 500.0f; // 积分限幅
    steering_pid_init(steering_kp, steering_ki, steering_kd, steering_imax);
}

/**
 * @brief  初始化单个串级PID控制器
 */
void DualPID_Init(DualPID_Controller *pid,
                 float pos_Kp, float pos_Ki, float pos_Kd,
                 float vel_Kp, float vel_Ki, float vel_Kd,
                 float max_vel, float max_output)
{
    pid->pos_Kp = pos_Kp;
    pid->pos_Ki = pos_Ki;
    pid->pos_Kd = pos_Kd;
    pid->vel_Kp = vel_Kp;
    pid->vel_Ki = vel_Ki;
    pid->vel_Kd = vel_Kd;
    pid->max_vel = max_vel;
    pid->max_output = max_output;
    pid->pos_integral = 0;
    pid->pos_error_prev = 0;
    pid->vel_integral = 0;
    pid->vel_error_prev = 0;
}

/**
 * @brief  初始化转向环PID控制器
 */
void steering_pid_init(float kp, float ki, float kd, float imax)
{
    pid_steering.kp = kp;
    pid_steering.ki = ki;
    pid_steering.kd = kd;
    pid_steering.imax = imax;
    pid_steering.integrator = 0.0f;
    pid_steering.last_error = 0.0f;
}


// =================================================================
// 3. PID计算与控制函数
// =================================================================
/**
 * @brief  更新串级PID控制器状态
 *
 * 串级PID结构（位置环在外层、速度环在内层）：
 *   目标位置 ──► [位置环PID] ──► 期望速度 ──► [速度环PID] ──► PWM输出
 *                  ▲                                ▲
 *              当前实际位置                     当前实际速度(编码器)
 *
 * 位置环：根据"位置误差"算出传给内环的"期望速度"；
 * 速度环：根据"速度误差"算出最终施加给电机的 PWM 输出。
 */
float DualPID_Update(DualPID_Controller *pid,
                    float target_pos, float current_pos,
                    float current_vel, float dt)
{
   //================ 位置环计算 ================
   // 1. 位置误差 = 目标位置 - 当前实际位置
    float pos_error = target_pos - current_pos;

   // 2. 位置环积分项：累计位置误差（Ki*dt 即积分步长），用于消除稳态位置误差
    pid->pos_integral += pid->pos_Ki * pos_error * dt;

   // 3. 位置环积分限幅：上限取"最大速度的23%"，防止积分饱和导致停车过冲/震荡
    float integral_limit_pos = pid->max_vel * 0.23f;
    if(pid->pos_integral > integral_limit_pos) pid->pos_integral = integral_limit_pos;
    if(pid->pos_integral < -integral_limit_pos) pid->pos_integral = -integral_limit_pos;

   // 4. 位置环微分项：位置误差的变化率(除以dt)，起到"阻尼/提前减速"作用，减少超调
    float pos_D = pid->pos_Kd * (pos_error - pid->pos_error_prev) / dt;

   // 5. 位置环输出 = P项 + I项 + D项，即交给速度环的"期望速度"
    float vel_target = pid->pos_Kp * pos_error + pid->pos_integral + pos_D;

   // 6. 期望速度限幅到 [-max_vel, +max_vel]，防止速度指令过大
    vel_target = fmaxf(fminf(vel_target, pid->max_vel), -pid->max_vel);

   // 7. 保存本次位置误差，供下一次微分计算使用
    pid->pos_error_prev = pos_error;

   //================ 速度环计算 ================
   // 1. 速度误差 = 期望速度 - 当前实际速度
    float vel_error = vel_target - current_vel;

   // 2. 速度环积分项：累计速度误差，用于消除稳态速度误差
    pid->vel_integral += pid->vel_Ki * vel_error * dt;

   // 3. 速度环积分限幅：上限取"最大输出的23%"，防止积分饱和
    float integral_limit_vel = pid->max_output * 0.23f;
    if (pid->vel_integral > integral_limit_vel) pid->vel_integral = integral_limit_vel;
    else if (pid->vel_integral < -integral_limit_vel) pid->vel_integral = -integral_limit_vel;

   // 4. 速度环微分项：速度误差的变化率，抑制速度突变/震荡
    float vel_D = pid->vel_Kd * (vel_error - pid->vel_error_prev) / dt;

   // 5. 速度环输出 = P项 + I项 + D项，即最终输出给电机的 PWM 值
    float output = pid->vel_Kp * vel_error + pid->vel_integral + vel_D;

   // 6. PWM 输出限幅到 [-max_output, +max_output]
    output = fmaxf(fminf(output, pid->max_output), -pid->max_output);

   // 7. 保存本次速度误差，供下一次微分计算使用
    pid->vel_error_prev = vel_error;

    return output;
}

/**
 * @brief  计算单环PID (用于转向)
 *
 * 位置式单环PID，专用于转向控制：
 *   输入 error = 目标值 - 当前值（例如：目标航向角 - 当前航向角）
 *   输出 out   = 转向控制量（例如：舵机角度 / 左右轮差速量）
 * 公式：out = Kp*error + Ki*Σerror + Kd*(error - last_error)
 */
float PidLocCtrl(pid_param_t * pid, float error)
{
	/* 1. 累积误差（积分项），用于消除转向的稳态误差 */
	pid->integrator += error;
	
    /* 2. 积分限幅：防止积分饱和导致转向震荡 */
    if (pid->integrator > pid->imax)   pid->integrator = pid->imax;
    if (pid->integrator < -pid->imax)  pid->integrator = -pid->imax;

	pid->out_p = pid->kp * error;   /* 3. P：比例项，误差越大输出越大 */
	pid->out_i = pid->ki * pid->integrator;   /*    I：积分项，消除稳态误差 */
	pid->out_d = pid->kd * (error - pid->last_error);   /*    D：微分项，抑制震荡 */
	
	pid->last_error = error;   /* 4. 保存本次误差，供下一次微分计算使用 */
	
    // 5. 三项求和得到最终输出
	pid->out = pid->out_p + pid->out_i + pid->out_d; 
	return pid->out;
}


/**
 * @brief  底盘四轮PID控制任务 (旧版，仅用于测试或不带转向的场合)
 */
void chassis_pid_control(int32_t target_pos_a, int32_t target_pos_b, 
                         int32_t target_pos_c, int32_t target_pos_d)
{
    get_encoder_data(&current_speed_a, &current_pos_a, &current_speed_b, &current_pos_b, &current_speed_c, &current_pos_c, &current_speed_d, &current_pos_d);

    float dt = 0.02f;

    // 对每个电机进行完整的串级PID计算
    int16_t pwm_a = (int16_t)DualPID_Update(&pid_motor_a, target_pos_a, current_pos_a, current_speed_a, dt);
    int16_t pwm_b = (int16_t)DualPID_Update(&pid_motor_b, target_pos_b, current_pos_b, current_speed_b, dt);
    int16_t pwm_c = (int16_t)DualPID_Update(&pid_motor_c, target_pos_c, current_pos_c, current_speed_c, dt);
    int16_t pwm_d = (int16_t)DualPID_Update(&pid_motor_d, target_pos_d, current_pos_d, current_speed_d, dt);

    motor_set_pwm(pwm_a, pwm_b, pwm_c, pwm_d);
}

