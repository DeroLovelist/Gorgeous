#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h" // 包含此文件以获取标准整数类型，如 int16_t, int32_t

/**
 * @brief  初始化所有电机编码器接口的时钟和模式
 * @param  None
 * @retval None
 */
void encoder_init(void);

/**
 * @brief  一次性读取所有四个编码器的数据
 * @note   此函数会读取每个编码器定时器自上次读取以来的增量值，
 *         这个增量值既是当前周期的“速度”，也用于累加到总位置上。
 *         读取后，硬件计数器会被清零，为下一个周期做准备。
 *
 * @param[out]  speed_a: 指向存储电机A速度的变量的指针
 * @param[out]  pos_a:   指向存储电机A累计位置的变量的指针
 * @param[out]  speed_b: 指向存储电机B速度的变量的指针
 * @param[out]  pos_b:   指向存储电机B累计位置的变量的指针
 * @param[out]  speed_c: 指向存储电机C速度的变量的指针
 * @param[out]  pos_c:   指向存储电机C累计位置的变量的指针
 * @param[out]  speed_d: 指向存储电机D速度的变量的指针
 * @param[out]  pos_d:   指向存储电机D累计位置的变量的指针
 * @retval None
 */
void get_encoder_data(int16_t *speed_a, int32_t *pos_a,
                      int16_t *speed_b, int32_t *pos_b,
                      int16_t *speed_c, int32_t *pos_c,
                      int16_t *speed_d, int32_t *pos_d);

#endif /* __ENCODER_H */
