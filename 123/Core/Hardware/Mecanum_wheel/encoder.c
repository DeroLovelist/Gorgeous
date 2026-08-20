#include "encoder.h"
#include "main.h" // 包含此文件以获取 htim2, htim3 等定时器句柄的定义

// 因为定时器句柄（如htim2）是在 main.c 中定义的，
// 所以在这里需要用 'extern' 关键字声明一下，告诉编译器这些变量在别处。
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/**
 * @brief  初始化所有电机编码器接口
 */
void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // 启动电机A (TIM2) 的编码器接口
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动电机B (TIM3) 的编码器接口
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动电机C (TIM4) 的编码器接口
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // 启动电机D (TIM5) 的编码器接口
}

/**
  * @brief  安全地读取所有编码器的增量值 (速度)，并累计总位置
  */
void get_encoder_data(int16_t *speed_a, int32_t *pos_a,
                      int16_t *speed_b, int32_t *pos_b,
                      int16_t *speed_c, int32_t *pos_c,
                      int16_t *speed_d, int32_t *pos_d)
{
    int16_t delta_a, delta_b, delta_c, delta_d;

    // --- 读取电机A (TIM2) ---
    delta_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim2); // 读取增量
    __HAL_TIM_SET_COUNTER(&htim2, 0);                 // 计数器清零
    *speed_a = delta_a;                              // 增量值就是当前速度
    *pos_a += delta_a;                               // 将增量累加到总位置

    // --- 读取电机B (TIM3) ---
    delta_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    *speed_b = delta_b;
    *pos_b += delta_b;

    // --- 读取电机C (TIM4) ---
    delta_c = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    *speed_c = delta_c;
    *pos_c += delta_c;

    // --- 读取电机D (TIM5) ---
    delta_d = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    *speed_d = delta_d;
    *pos_d += delta_d;
}
