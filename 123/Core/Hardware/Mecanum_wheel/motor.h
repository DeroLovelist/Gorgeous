#ifndef __MOTOR_H
#define __MOTOR_H

#include "stdint.h" // 使用标准库的int16_t

void motor_init(void);
void motor_set_pwm(int16_t pwm_a, int16_t pwm_b, int16_t pwm_c, int16_t pwm_d);

#endif
