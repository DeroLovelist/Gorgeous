#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

#define PWM_ARR		99	/* TIM8 自动重装值(占空比范围 0~99) */

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void PWM_SetCompare3(uint16_t Compare);
void PWM_SetCompare4(uint16_t Compare);

#endif
