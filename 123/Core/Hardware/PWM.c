#include "main.h"
#include "tim.h"
#include "PWM.h"

/*
 * TIM8 PWM: PSC=83, ARR=99 -> 168MHz/(84*100) = 20kHz
 * 四通道: CH1=PC6(PWMA2)  CH2=PC7(PWMB2)  CH3=PC8(PWMB1)  CH4=PC9(PWMA1)
 * 定时器时基与通道配置由 MX_TIM8_Init 完成, 这里只需启动输出
 */
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

//设置 PWM 占空比 (0~100)
void PWM_SetCompare1(uint16_t Compare)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Compare);
}

void PWM_SetCompare2(uint16_t Compare)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Compare);
}

void PWM_SetCompare3(uint16_t Compare)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Compare);
}

void PWM_SetCompare4(uint16_t Compare)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, Compare);
}
