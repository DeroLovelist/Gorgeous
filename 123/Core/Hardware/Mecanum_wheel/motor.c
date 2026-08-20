/*
(车头 Front)
      /-----------\
     /             \
  电机A ---------- 电机B
(前左 FL)      (前右 FR)
  滚子 \         / 滚子
       |         |
       | Robot   |
       |         |
  滚子 /         \ 滚子
  电机C ---------- 电机D
(后左 BL)      (后右 BR)
     \             /
      \-----------/
      (车尾 Rear)
*/






#include "main.h" 
#include "motor.h"
 
                   
//#include "tim.h"// CubeMX为定时器生成的头文件，必须包含
// 初始化函数
void motor_init(void)
{
    // 启动4个PWM通道
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // Motor A -> PC6
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // Motor B -> PC7
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // Motor C -> PC8
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // Motor D -> PC9
}

// 辅助函数，用于控制单个电机
static void set_single_motor_pwm(int16_t pwm, uint32_t channel, 
                                 GPIO_TypeDef* port1, uint16_t pin1, 
                                 GPIO_TypeDef* port2, uint16_t pin2)
{
    if (pwm > 0)
	{ // 正转
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, channel, pwm);
    } 
	else if (pwm < 0) 
	{ // 反转
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, channel, -pwm);
    } 
	else 
	{ // 停止 (滑行)
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, channel, 0);
    }
}

// 设置4个电机PWM的函数
void motor_set_pwm(int16_t pwm_a, int16_t pwm_b, int16_t pwm_c, int16_t pwm_d)
{
    // Motor A (AIN1/2 -> PE2/3, PWM -> CH1)
    set_single_motor_pwm(pwm_a, TIM_CHANNEL_1, AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin);

    // Motor B (BIN1/2 -> PE4/5, PWM -> CH2)
    set_single_motor_pwm(pwm_b, TIM_CHANNEL_2, BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);

    // Motor C (CIN1/2 -> PC0/1, PWM -> CH3)
    set_single_motor_pwm(pwm_c, TIM_CHANNEL_3, CIN1_GPIO_Port, CIN1_Pin, CIN2_GPIO_Port, CIN2_Pin);
    
    // Motor D (DIN1/2 -> PC2/3, PWM -> CH4)
    set_single_motor_pwm(pwm_d, TIM_CHANNEL_4, DIN1_GPIO_Port, DIN1_Pin, DIN2_GPIO_Port, DIN2_Pin);
}


