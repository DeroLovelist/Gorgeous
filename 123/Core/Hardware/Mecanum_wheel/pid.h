#ifndef __PID_H
#define __PID_H

// 建议包含一个包含基本类型定义的头文件，例如 stdint.h 或您工程的 main.h
#include <stdint.h>

// =================================================================
// 1. 结构体定义
// =================================================================

/**
 * @brief 串级PID控制器结构体 (用于电机的位置环+速度环)
 */
typedef struct
{
    // 位置环参数
    float pos_Kp, pos_Ki, pos_Kd;
    float pos_integral;        // 位置环积分累计值
    float pos_error_prev;      // 上一次的位置误差

    // 速度环参数
    float vel_Kp, vel_Ki, vel_Kd;
    float vel_integral;        // 速度环积分累计值
    float vel_error_prev;      // 上一次的速度误差

    // 限幅参数
    float max_vel;     // 位置环输出的最大目标速度
    float max_output;  // 速度环输出的最大PWM值或等效值

} DualPID_Controller;

/**
 * @brief 单环PID控制器结构体 (用于转向环)
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float imax;        // 积分项限幅
    float integrator;
    float last_error;
    float out_p;
    float out_i;
    float out_d;
    float out;
} pid_param_t;


// =================================================================
// 2. 外部变量声明
// =================================================================

// 声明四个电机的串级PID控制器实例
extern DualPID_Controller pid_motor_a;
extern DualPID_Controller pid_motor_b;
extern DualPID_Controller pid_motor_c;
extern DualPID_Controller pid_motor_d;

// 声明转向环PID控制器实例
extern pid_param_t pid_steering;


// =================================================================
// 3. 函数声明
// =================================================================

/**
 * @brief 初始化所有PID控制器 (电机串级PID + 转向环PID)
 */
void pid_init_all(void);

/**
 * @brief 初始化单个电机的串级PID控制器
 */
void DualPID_Init(DualPID_Controller *pid,
                 float pos_Kp, float pos_Ki, float pos_Kd,
                 float vel_Kp, float vel_Ki, float vel_Kd,
                 float max_vel, float max_output);

/**
 * @brief 更新单个电机的串级PID计算
 * @return 计算得到的PWM输出值
 */
float DualPID_Update(DualPID_Controller *pid,
                    float target_pos, float current_pos,
                    float current_vel, float dt);

/**
 * @brief 【旧】底盘四轮PID控制任务 (仅位置控制)
 */
void chassis_pid_control(int32_t target_pos_a, int32_t target_pos_b, 
                         int32_t target_pos_c, int32_t target_pos_d);

/**
 * @brief 计算单环PID (用于转向)
 * @return PID计算输出
 */
float PidLocCtrl(pid_param_t *pid, float error);

/**
 * @brief 初始化转向环PID控制器
 */
void steering_pid_init(float kp, float ki, float kd, float imax);


#endif /* __PID_H */

