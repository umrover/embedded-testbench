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
#define Heater_0_Pin GPIO_PIN_1
#define Heater_0_GPIO_Port GPIOF
#define Heater_2_Pin GPIO_PIN_0
#define Heater_2_GPIO_Port GPIOC
#define Heater_1_Pin GPIO_PIN_1
#define Heater_1_GPIO_Port GPIOC
#define Ammonia_FWD_Pin GPIO_PIN_2
#define Ammonia_FWD_GPIO_Port GPIOC
#define DEBUG_UART_TX_Pin GPIO_PIN_2
#define DEBUG_UART_TX_GPIO_Port GPIOA
#define DEBUG_UART_RX_Pin GPIO_PIN_3
#define DEBUG_UART_RX_GPIO_Port GPIOA
#define Therm_0_Pin GPIO_PIN_4
#define Therm_0_GPIO_Port GPIOA
#define Ammonia_PWM_Pin GPIO_PIN_6
#define Ammonia_PWM_GPIO_Port GPIOA
#define Therm_1_Pin GPIO_PIN_1
#define Therm_1_GPIO_Port GPIOB
#define Ammonia_BWD_Pin GPIO_PIN_10
#define Ammonia_BWD_GPIO_Port GPIOB
#define Therm_2_Pin GPIO_PIN_12
#define Therm_2_GPIO_Port GPIOB
#define Pump_2_Pin GPIO_PIN_15
#define Pump_2_GPIO_Port GPIOB
#define Auton_Green_LED_Pin GPIO_PIN_6
#define Auton_Green_LED_GPIO_Port GPIOC
#define Auton_Blue_LED_Pin GPIO_PIN_7
#define Auton_Blue_LED_GPIO_Port GPIOC
#define Auton_Red_LED_Pin GPIO_PIN_8
#define Auton_Red_LED_GPIO_Port GPIOC
#define Mux_SCL_Pin GPIO_PIN_9
#define Mux_SCL_GPIO_Port GPIOA
#define Mux_SDA_Pin GPIO_PIN_10
#define Mux_SDA_GPIO_Port GPIOA
#define sci_UV_LED_Pin GPIO_PIN_10
#define sci_UV_LED_GPIO_Port GPIOC
#define SA_UV_LED_Pin GPIO_PIN_11
#define SA_UV_LED_GPIO_Port GPIOC
#define Pump_0_Pin GPIO_PIN_12
#define Pump_0_GPIO_Port GPIOC
#define J_UART_TX_Pin GPIO_PIN_6
#define J_UART_TX_GPIO_Port GPIOB
#define J_UART_RX_Pin GPIO_PIN_7
#define J_UART_RX_GPIO_Port GPIOB
#define Pump_1_Pin GPIO_PIN_8
#define Pump_1_GPIO_Port GPIOB
#define whiteLED_Pin GPIO_PIN_9
#define whiteLED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
