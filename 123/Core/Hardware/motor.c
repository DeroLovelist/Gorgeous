#include "main.h"
#include "tim.h"
#include "PWM.h"
#include "motor.h"

/*
 * 电机驱动引脚映射 (main.h 中由 CubeMX 定义):
 *
 *   电机1: AIN1_1(PA6)  + AIN1_2(PA4)   PWMA1 = TIM8_CH4(PC9)   编码器 TIM1(PE9/PE11)
 *   电机2: BIN1_1(PC4)  + BIN1_2(PB0)   PWMB1 = TIM8_CH3(PC8)   编码器 TIM5(PA0/PA1)
 *   电机3: AIN2_1(PA15) + AIN2_2(PD1)   PWMA2 = TIM8_CH1(PC6)   编码器 TIM4(PD12/PD13)  方向已反转
 *   电机4: BIN2_1(PA8)  + BIN2_2(PD0)   PWMB2 = TIM8_CH2(PC7)   编码器 TIM3(PB4/PB5)  方向已反转
 *
 *   PWM 频率 20kHz, 占空比范围 0~99 (TIM8: PSC=83, ARR=99)
 *   M3/M4 在 Set_PWM 中交换 in1/in2, 使正 PWM 对应物理转向与 M1/M2 一致
 */
void Motor_Init(void)
{
	PWM_Init();			/* 启动 TIM8 四路 PWM */
	Encoder_Init();		/* 启动四个编码器 (ENCODER 模块, 见 ENCODER.c) */
}

/*
 * 设置电机 PWM 与方向
 * 入口参数: n=1..4 电机号; PWM>0 正转, PWM<0 反转, 范围 -PWM_ARR~PWM_ARR
 * 方向约定: 第一个方向引脚=1 且第二个=0 为正转(PWM>=0)
 * M3/M4 为反转配置: 写入时 in1/in2 互换
 */
void Set_PWM(uint8_t n, int16_t PWM)
{
	GPIO_PinState in1, in2;
	uint16_t duty;

	if (PWM >  PWM_ARR) PWM =  PWM_ARR;
	if (PWM < -PWM_ARR) PWM = -PWM_ARR;

	in1  = (PWM >= 0) ? GPIO_PIN_SET   : GPIO_PIN_RESET;
	in2  = (PWM >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
	duty = (PWM >= 0) ? (uint16_t)PWM : (uint16_t)(-PWM);

	switch (n)
	{
	case MOTOR_1:	/* AIN1_1 + AIN1_2, PWMA1(TIM8_CH4) */
		HAL_GPIO_WritePin(AIN1_1_GPIO_Port, AIN1_1_Pin, in1);
		HAL_GPIO_WritePin(AIN1_2_GPIO_Port, AIN1_2_Pin, in2);
		PWM_SetCompare4(duty);
		break;
	case MOTOR_2:	/* BIN1_1 + BIN1_2, PWMB1(TIM8_CH3) */
		HAL_GPIO_WritePin(BIN1_1_GPIO_Port, BIN1_1_Pin, in1);
		HAL_GPIO_WritePin(BIN1_2_GPIO_Port, BIN1_2_Pin, in2);
		PWM_SetCompare3(duty);
		break;
	case MOTOR_3:	/* AIN2_1 + AIN2_2, PWMA2(TIM8_CH1)  方向已反转: 交换 in1/in2 */
		HAL_GPIO_WritePin(AIN2_1_GPIO_Port, AIN2_1_Pin, in2);
		HAL_GPIO_WritePin(AIN2_2_GPIO_Port, AIN2_2_Pin, in1);
		PWM_SetCompare1(duty);
		break;
	case MOTOR_4:	/* BIN2_1 + BIN2_2, PWMB2(TIM8_CH2)  方向已反转: 交换 in1/in2 */
		HAL_GPIO_WritePin(BIN2_1_GPIO_Port, BIN2_1_Pin, in2);
		HAL_GPIO_WritePin(BIN2_2_GPIO_Port, BIN2_2_Pin, in1);
		PWM_SetCompare2(duty);
		break;
	default:
		break;
	}
}

