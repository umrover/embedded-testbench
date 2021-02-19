/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Ammonia_FWD_Pin GPIO_PIN_14
#define Ammonia_FWD_GPIO_Port GPIOC
#define Ammonia_BWD_Pin GPIO_PIN_10
#define Ammonia_BWD_GPIO_Port GPIOB
#define Auton_Green_LED_Pin GPIO_PIN_6
#define Auton_Green_LED_GPIO_Port GPIOC
#define Auton_Blue_LED_Pin GPIO_PIN_7
#define Auton_Blue_LED_GPIO_Port GPIOC
#define Auton_Red_LED_Pin GPIO_PIN_8
#define Auton_Red_LED_GPIO_Port GPIOC
#define sci_UV_LED_Pin GPIO_PIN_10
#define sci_UV_LED_GPIO_Port GPIOC
#define SA_UV_LED_Pin GPIO_PIN_11
#define SA_UV_LED_GPIO_Port GPIOC
#define Pump_2_Pin GPIO_PIN_12
#define Pump_2_GPIO_Port GPIOC
#define Pump_1_Pin GPIO_PIN_8
#define Pump_1_GPIO_Port GPIOB
#define Pump_0_Pin GPIO_PIN_9
#define Pump_0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
