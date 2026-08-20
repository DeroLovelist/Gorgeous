/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Light_Pin GPIO_PIN_13
#define Light_GPIO_Port GPIOC
#define T5C1_Pin GPIO_PIN_0
#define T5C1_GPIO_Port GPIOA
#define T5C2_Pin GPIO_PIN_1
#define T5C2_GPIO_Port GPIOA
#define AIN1_2_Pin GPIO_PIN_4
#define AIN1_2_GPIO_Port GPIOA
#define AIN1_1_Pin GPIO_PIN_6
#define AIN1_1_GPIO_Port GPIOA
#define BIN1_1_Pin GPIO_PIN_4
#define BIN1_1_GPIO_Port GPIOC
#define BIN1_2_Pin GPIO_PIN_0
#define BIN1_2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define T1C1_Pin GPIO_PIN_9
#define T1C1_GPIO_Port GPIOE
#define T1C2_Pin GPIO_PIN_11
#define T1C2_GPIO_Port GPIOE
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOB
#define BLU_TX_Pin GPIO_PIN_8
#define BLU_TX_GPIO_Port GPIOD
#define BLU_RX_Pin GPIO_PIN_9
#define BLU_RX_GPIO_Port GPIOD
#define T4C1_Pin GPIO_PIN_12
#define T4C1_GPIO_Port GPIOD
#define T4C2_Pin GPIO_PIN_13
#define T4C2_GPIO_Port GPIOD
#define PWMA2_Pin GPIO_PIN_6
#define PWMA2_GPIO_Port GPIOC
#define PWMB2_Pin GPIO_PIN_7
#define PWMB2_GPIO_Port GPIOC
#define PWMB1_Pin GPIO_PIN_8
#define PWMB1_GPIO_Port GPIOC
#define PWMA1_Pin GPIO_PIN_9
#define PWMA1_GPIO_Port GPIOC
#define BIN2_1_Pin GPIO_PIN_8
#define BIN2_1_GPIO_Port GPIOA
#define AIN2_1_Pin GPIO_PIN_15
#define AIN2_1_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOC
#define BIN2_2_Pin GPIO_PIN_0
#define BIN2_2_GPIO_Port GPIOD
#define AIN2_2_Pin GPIO_PIN_1
#define AIN2_2_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOD
#define T3C1_Pin GPIO_PIN_4
#define T3C1_GPIO_Port GPIOB
#define T3C2_Pin GPIO_PIN_5
#define T3C2_GPIO_Port GPIOB
#define JY_SCL_Pin GPIO_PIN_6
#define JY_SCL_GPIO_Port GPIOB
#define JY_SDA_Pin GPIO_PIN_7
#define JY_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
