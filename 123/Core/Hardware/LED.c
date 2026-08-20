#include "main.h"
#include "LED.h"

/**
 * @brief  LED 初始化 (PB2 已由 MX_GPIO_Init 配置为推挽输出), 初始化为熄灭
 */
void LED_Init(void)
{
	LED_OFF();
}

/* LED 为高电平点亮 (PB2=1 亮) */
void LED_ON(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void LED_OFF(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_Turn(void)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
