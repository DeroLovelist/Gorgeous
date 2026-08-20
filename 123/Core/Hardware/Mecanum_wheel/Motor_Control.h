#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" // Or your primary header file for types like int32_t
	#include <stdbool.h>
	#include <stdlib.h> // ★★★ 添加这一行 ★★★

// ================== 底盘物理参数宏定义 ==================

// 1. 编码器精度：电机输出轴转一圈对应的编码器总脉冲数
//    您提供的数据是 65000
#define ENCODER_PULSES_PER_ROUND    65000.0f

// 2. 轮子周长 (单位: 毫米 mm)
//    请用尺子测量您的麦轮直径，然后乘以 3.14159f
//    例如：直径80mm -> 80.0f * 3.14159f = 251.327f
#define WHEEL_PERIMETER_MM          251.327f  // ★★★ 请务必修改为您的轮子周长 ★★★

// 3. 脉冲到毫米的转换系数
//    1个脉冲代表轮子前进了多少毫米
#define PULSE_TO_MM                 (WHEEL_PERIMETER_MM / ENCODER_PULSES_PER_ROUND)


// ================== 运动控制函数声明 ==================

/**
 * @brief  初始化电机控制模块
 */
void Motor_Control_Init(void);

// --- 平移运动指令 (原函数保持不变) ---

/**
 * @brief  让底盘向前移动指定的距离
 */
void Chassis_Move_Forward(int32_t distance_mm);

/**
 * @brief  让底盘向后移动指定的距离
 */
void Chassis_Move_Backward(int32_t distance_mm);

/**
 * @brief  让底盘向左平移指定的距离
 */
void Chassis_Move_Left(int32_t distance_mm);

/**
 * @brief  让底盘向右平移指定的距离
 */
void Chassis_Move_Right(int32_t distance_mm);



/**
 * @brief  让底盘向左上45度方向移动指定的距离
 * @note   物理动作: A轮(前进), D轮(前进), B轮(不动), C轮(不动)
 *         对应代码语言: A(+), D(-)
 * @param  distance_mm 要移动的距离 (单位: 毫米)
 */
void Chassis_Move_TopLeft(int32_t distance_mm);



/**
 * @brief  停止所有运动，将目标位置设置为当前位置
 */
void Chassis_Stop(void);

// --- 【新增】转向运动指令 ---

/**
 * @brief  启动转向控制，使底盘旋转到指定的目标角度
 * @param  Angle: 目标角度 (范围: -180 到 180)
 */
void Turn_Angle(float Angle);


// --- 核心更新函数 (原函数保持不变) ---

/**
 * @brief  【核心】运动学解算与PID控制任务
 * @note   此函数应在固定周期的定时器中断中被调用
 */
void Chassis_Update_Control(void);

bool Chassis_Task_Is_Complete(void);


#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */

