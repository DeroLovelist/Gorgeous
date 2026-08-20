#ifndef __LASER_H__
#define __LASER_H__

/*
 * LASER 激光模块
 *
 * 硬件连接:
 *   激光引脚 = CubeMX 中的 Light (PC13), 高电平触发点亮
 *
 * MX 配置:
 *   Light_Pin = GPIO_PIN_13
 *   Light_GPIO_Port = GPIOC
 *   推挽输出, 下拉, 初始低电平 (关闭)
 */

#include "main.h"

/* 激光引脚使用 CubeMX 生成的 Light (PC13) 别名 */
#define LASER_Pin         Light_Pin
#define LASER_GPIO_Port   Light_GPIO_Port

void Laser_Init(void);
void Laser_On(void);
void Laser_Off(void);
void Laser_SetState(uint8_t state);

#endif /* __LASER_H__ */
