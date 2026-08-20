#ifndef __ENCODER_H__
#define __ENCODER_H__

/*
 * ENCODER 电机编码器模块 — 适配四路电机编码器 (TIM1/TIM5/TIM4/TIM3)
 *
 * 硬件连接 (CubeMX 已配置为编码器模式, 引脚为复用功能 AF):
 *   电机1: TIM1 (PE9/PE11, AF1)        电机2: TIM5 (PA0/PA1, AF2)
 *   电机3: TIM4 (PD12/PD13, AF2)       电机4: TIM3 (PB4/PB5, AF2)
 *
 * 配置 (由 MX 生成):
 *   - 编码器模式: TIM_ENCODERMODE_TI12 (4倍频, CH1+CH2 双沿计数)
 *   - 计数器周期: TIM1/TIM5 为 32 位, TIM3/TIM4 为 16 位
 *   - 分频: 0 (不分频)   极性: 上升沿 (Rising)
 *
 * 与 motor 模块的关系 (功能分开、相辅相成):
 *   - 本模块只负责编码器: 初始化 / 读取 / 清零 / 增量 / 速度 / 步数
 *   - motor 模块只负责 PWM 与方向; 其 Motor_Init() 会调用本模块的
 *     Encoder_Init() 统一启动四路编码器
 *
 * 使用说明:
 *   1. 所有接口以电机号 n(1~4) 作为参数, 与 MOTOR_1~MOTOR_4 同值
 *   2. 上电后在 main() 中调用 Motor_Init() (或直接调用 Encoder_Init()) 即可
 *   3. Encoder_GetDelta() 需周期性调用才会刷新内部速度 / 步数
 *   4. 原始计数为 4 倍频: 每物理格 = ENCODER_PPR(4) 个计数
 */

#include <stdint.h>
#include "main.h"
#include "tim.h" /* htim1/htim3/htim4/htim5 */

/* ==================== 4倍频系数 ==================== */
#define ENCODER_PPR 4 /* TI12 模式下每物理格 = 4 个计数 */

/* ==================== 编码器通道(内部索引) ==================== */
typedef enum
{
	ENC_CH1 = 0, /* TIM1 -> 电机1 (PE9/PE11)  */
	ENC_CH2 = 1, /* TIM5 -> 电机2 (PA0/PA1)   */
	ENC_CH3 = 2, /* TIM4 -> 电机3 (PD12/PD13) */
	ENC_CH4 = 3, /* TIM3 -> 电机4 (PB4/PB5)   */
	ENC_CH_MAX
} ENC_Channel_t;

/* ==================== API ==================== */

/**
 * @brief  配置四路编码器 GPIO(复用功能) 并启动计数
 * @note   通常由 Motor_Init() 调用; 也可单独调用
 */
void Encoder_Init(void);

/**
 * @brief  启动指定电机编码器计数
 * @param  n 电机号 1~4 (与 MOTOR_1~MOTOR_4 同值)
 */
void Encoder_Start(uint8_t n);

/**
 * @brief  停止指定电机编码器计数
 * @param  n 电机号 1~4
 */
void Encoder_Stop(uint8_t n);

/**
 * @brief  读取指定电机编码器原始计数值
 * @param  n 电机号 1~4
 * @return 当前计数值, 溢出自动回绕 (TIM1/TIM5 为 32 位, TIM3/TIM4 为 16 位)
 * @note   4倍频模式下, 每圈计数 = 编码器线数 × 4
 */
uint32_t Encoder_Read(uint8_t n);

/**
 * @brief  读取指定电机编码器有符号计数值
 * @param  n 电机号 1~4
 * @return 有符号计数值 (等价于原 Motor_GetEncoderCount 接口)
 */
int32_t Encoder_GetCount(uint8_t n);

/**
 * @brief  获取本次读取相对于上次的变化量
 * @param  n 电机号 1~4
 * @return 有符号增量, 正值=正转(CW), 负值=反转(CCW)
 * @note   内部按计数器宽度(16/32 位)自动处理溢出回绕
 */
int16_t Encoder_GetDelta(uint8_t n);

/**
 * @brief  获取指定电机编码器转速 (count/s)
 * @param  n 电机号 1~4
 * @return 每秒计数变化量, 正值=正转, 负值=反转
 * @note   需周期性调用 Encoder_GetDelta() / Encoder_Read() 刷新
 */
int32_t Encoder_GetSpeed(uint8_t n);

/**
 * @brief  清零指定电机编码器硬件计数器
 * @param  n 电机号 1~4
 */
void Encoder_ResetCount(uint8_t n);

/**
 * @brief  清零指定电机编码器计数器并重置内部软件状态
 * @param  n 电机号 1~4
 */
void Encoder_Reset(uint8_t n);

/**
 * @brief  获取指定电机编码器步数增量 (已除 4, 1 步 = 1 个物理格)
 * @param  n 电机号 1~4
 * @return 有符号步数增量, >0=正转, <0=反转
 * @note   内部调用 Encoder_GetDelta() 后除以 ENCODER_PPR
 */
int16_t Encoder_GetStepDelta(uint8_t n);

/**
 * @brief  获取指定电机编码器当前累计步数 (已除 4)
 * @param  n 电机号 1~4
 * @return 累计步数
 */
int32_t Encoder_GetStep(uint8_t n);

#endif /* __ENCODER_H__ */
