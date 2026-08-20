#include "motor_control.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include <math.h> 
#include <stdbool.h>
#include <stdlib.h> // 包含 stdlib.h，以使用 abs() 求绝对值

// ================== 外部变量声明 ==================
extern DualPID_Controller pid_motor_a, pid_motor_b, pid_motor_c, pid_motor_d;
extern pid_param_t pid_steering;
extern int16_t current_speed_a, current_speed_b, current_speed_c, current_speed_d;
extern int32_t current_pos_a, current_pos_b, current_pos_c, current_pos_d;
extern float global_angle;

// ================== 内部静态变量定义 ==================
static int32_t target_pos_a = 0;
static int32_t target_pos_b = 0;
static int32_t target_pos_c = 0;
static int32_t target_pos_d = 0;

static volatile uint8_t Turn_OPEN = 0;
static float Targt_angle = 0.0f;
static uint16_t Turn_count = 0;

static volatile bool g_chassis_is_moving = false; 

#define POSITION_THRESHOLD 100 // 位置误差判断阈值：各轮目标位置与当前位置相差小于该值即认为到位
#define PULSE_TO_MM 100.0f     // 毫米转脉冲的换算系数（每 1mm 对应的编码器脉冲数，需按实际轮子标定）

// ================== 底盘任务完成判断 ==================
bool Chassis_Task_Is_Complete(void)
{
    if (Turn_OPEN || g_chassis_is_moving)
    {
        return false;
    }
    return true;
}

// ================== 电机控制模块初始化 ==================
void Motor_Control_Init(void)
{
    get_encoder_data(&current_speed_a, &current_pos_a, &current_speed_b, &current_pos_b,
                     &current_speed_c, &current_pos_c, &current_speed_d, &current_pos_d);
    target_pos_a = current_pos_a;
    target_pos_b = current_pos_b;
    target_pos_c = current_pos_c;
    target_pos_d = current_pos_d;
}

static inline int32_t mm_to_pulses(int32_t mm)
{
    return (int32_t)((float)mm * PULSE_TO_MM); // 将毫米距离换算为编码器脉冲数
}

static inline float myfabs(float val)
{
    return val > 0 ? val : -val;
}

float Yaw_error(float Target, float Now)
{
    float error = Target - Now;
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    return error;
}

// ================== 底盘平移运动指令（麦克纳姆轮各轮转向组合）==================

// 前进：A+, B-, C+, D-（四个轮子协同实现正向前进）
void Chassis_Move_Forward(int32_t distance_mm)
{
    g_chassis_is_moving = true;
    int32_t pulses = mm_to_pulses(distance_mm);
    target_pos_a += pulses;
    target_pos_b -= pulses;
    target_pos_c += pulses;
    target_pos_d -= pulses;
}

// 后退：A-, B+, C-, D+（与前进方向相反的轮子转向）
void Chassis_Move_Backward(int32_t distance_mm)
{
    g_chassis_is_moving = true;
    int32_t pulses = mm_to_pulses(distance_mm);
    target_pos_a -= pulses;
    target_pos_b += pulses;
    target_pos_c -= pulses;
    target_pos_d += pulses;
}

// 左移：A-, B-, C+, D+（横向平移）
void Chassis_Move_Left(int32_t distance_mm)
{
    g_chassis_is_moving = true;
    int32_t pulses = mm_to_pulses(distance_mm);
    target_pos_a -= pulses;
    target_pos_b -= pulses;
    target_pos_c += pulses;
    target_pos_d += pulses;
}

// 右移：A+, B+, C-, D-（横向平移）
void Chassis_Move_Right(int32_t distance_mm)
{
    g_chassis_is_moving = true;
    int32_t pulses = mm_to_pulses(distance_mm);
    target_pos_a += pulses;
    target_pos_b += pulses;
    target_pos_c -= pulses;
    target_pos_d -= pulses;
}

// 左上45度斜向移动 (TopLeft) = 前进 + 左移 的合成运动
// 前进时各轮符号: (+, -, +, -)
// 左移时各轮符号: (-, -, +, +)
// 合成结果: (0, -2, +2, 0) -> 即 A/D 轮不动，只让 B 轮反向、C 轮正向
void Chassis_Move_TopLeft(int32_t distance_mm)
{
    g_chassis_is_moving = true;
    int32_t pulses = mm_to_pulses(distance_mm); 
    // 仅 B 轮反向、C 轮正向（A、D 轮保持不动），实现左上45度斜向平移
    target_pos_b -= pulses; // B 轮目标位置减小（反向转动）
    target_pos_c += pulses; // C 轮目标位置增大（正向转动）
}

void Chassis_Stop(void)
{
    g_chassis_is_moving = false;
    Turn_OPEN = 0;
    target_pos_a = current_pos_a;
    target_pos_b = current_pos_b;
    target_pos_c = current_pos_c;
    target_pos_d = current_pos_d;
}

void Turn_Angle(float Angle)
{
    Targt_angle = Angle;
    Turn_count = 0;
    Turn_OPEN = 1;
}

// ================== 底盘控制更新函数（应在定时器中断里周期性调用）==================
void Chassis_Update_Control(void)
{
    // --- 1. 读取编码器（更新各轮当前速度与位置）---
    get_encoder_data(&current_speed_a, &current_pos_a, &current_speed_b, &current_pos_b, 
                     &current_speed_c, &current_pos_c, &current_speed_d, &current_pos_d);

    // --- 2. 转向控制（陀螺仪闭环）---
    if (Turn_OPEN)
    {
        float error = Yaw_error(Targt_angle, global_angle);  // 计算目标角度与当前角度的误差
        float steering_adjustment = PidLocCtrl(&pid_steering, error);  // PID 计算转向修正量
        
        const float MAX_TURN_ADJUSTMENT = 150.0f;
        if (steering_adjustment > MAX_TURN_ADJUSTMENT) steering_adjustment = MAX_TURN_ADJUSTMENT;
        if (steering_adjustment < -MAX_TURN_ADJUSTMENT) steering_adjustment = -MAX_TURN_ADJUSTMENT;
        
        int32_t adjustment = (int32_t)steering_adjustment;
        
        // 转向修正：把 PID 输出的修正量同步加到四个轮子的目标位置上
        // 当 adjustment > 0（需要朝某个方向旋转/陀螺仪修正）时，
        // 四个轮子统一减去该修正量 (A-, B-, C-, D-)，实现原地旋转；
        // adjustment 的正负决定了旋转方向。
        // （把 adjustment 加到/减去 target，让各轮目标位置同步偏移，从而产生转向动作）
        
        target_pos_a -= adjustment;
        target_pos_b -= adjustment; 
        target_pos_c -= adjustment;
        target_pos_d -= adjustment;

        // 角度稳定判定：连续若干周期误差都足够小，才认为转向到位
        const float ANGLE_ERROR_THRESHOLD = 2.0f;    // 角度误差阈值
        const uint16_t STABLE_COUNT_THRESHOLD = 5;

        if (myfabs(error) < ANGLE_ERROR_THRESHOLD)
        {
            if (++Turn_count >= STABLE_COUNT_THRESHOLD)
            {
                Turn_OPEN = 0;
                Turn_count = 0;
                Chassis_Stop();
            }
        }
        else
        {
            Turn_count = 0;
        }
    }
    // --- 3. 平移/斜移到位判断 ---
    else if (g_chassis_is_moving)
    {
        if (abs(target_pos_a - current_pos_a) < POSITION_THRESHOLD &&
            abs(target_pos_b - current_pos_b) < POSITION_THRESHOLD &&
            abs(target_pos_c - current_pos_c) < POSITION_THRESHOLD &&
            abs(target_pos_d - current_pos_d) < POSITION_THRESHOLD)
        {
            g_chassis_is_moving = false; 
        }
    }
    
    // --- 4. PID 闭环输出各轮 PWM ---
    float dt = 0.02f; 
    int16_t pwm_a = (int16_t)DualPID_Update(&pid_motor_a, target_pos_a, current_pos_a, current_speed_a, dt);
    int16_t pwm_b = (int16_t)DualPID_Update(&pid_motor_b, target_pos_b, current_pos_b, current_speed_b, dt);
    int16_t pwm_c = (int16_t)DualPID_Update(&pid_motor_c, target_pos_c, current_pos_c, current_speed_c, dt);
    int16_t pwm_d = (int16_t)DualPID_Update(&pid_motor_d, target_pos_d, current_pos_d, current_speed_d, dt);

    motor_set_pwm(pwm_a, pwm_b, pwm_c, pwm_d);
}

