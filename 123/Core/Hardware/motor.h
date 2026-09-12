#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "ENCODER.h" /* 编码器模块: Motor_Init() 调用 Encoder_Init() */

#define MOTOR_1		1
#define MOTOR_2		2
#define MOTOR_3		3
#define MOTOR_4		4

void Motor_Init(void);
void Set_PWM(uint8_t n, int16_t PWM);

#endif
