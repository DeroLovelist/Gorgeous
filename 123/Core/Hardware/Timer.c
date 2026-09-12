#include "main.h"
#include "tim.h"
#include "Timer.h"
#include "Key.h"
#include "Chassis.h"

/*
 * 1ms 定时器: TIM9 (PSC=999, ARR=167 -> 168MHz/1000/168 = 1kHz = 1ms)
 * 定时器与 NVIC 由 MX_TIM9_Init 配置, 这里启动更新中断
 */
void Timer_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim9);	/* 使能更新中断并启动 */
}

/* TIM9 每 1ms 进入一次; 这里把 Key_Tick 分频为每 10ms 执行一次 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t keyTickDiv = 0;   /* 1ms 中断计数 */

	if (htim == &htim9)
	{
		if (++keyTickDiv >= 10)
		{
			keyTickDiv = 0;
			Key_Tick();		/* 10ms 按键扫描 */
		}

		Chassis_Tick();		/* 底盘控制: 内部每 20ms 执行一次 */
	}
}
