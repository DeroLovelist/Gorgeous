#include "LASER.h"

/**
 * @brief  激光模块初始化 (GPIO 已由 MX 配置, 这里仅安全关断)
 */
void Laser_Init(void)
{
    Laser_Off();
}

/**
 * @brief  打开激光 (PB2 输出高电平)
 */
void Laser_On(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}

/**
 * @brief  关闭激光 (PB2 输出低电平)
 */
void Laser_Off(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  按状态控制激光
 * @param  state  非 0=开, 0=关
 */
void Laser_SetState(uint8_t state)
{
    if (state)
        Laser_On();
    else
        Laser_Off();
}
