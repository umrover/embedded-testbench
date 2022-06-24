/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//Global buffer for receiving UART commands
extern volatile uint8_t Rx_data[30];
//Global buffer of the last received command.
extern volatile uint8_t message[30];

extern UART_HandleTypeDef huart1;
// Debugging UART through usb connection
extern UART_HandleTypeDef huart2;

// enable servo's TIM
extern TIM_HandleTypeDef htim1;
// enable carousel's TIM
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
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
#define LED_BLUE_DEBUG_Pin GPIO_PIN_1
#define LED_BLUE_DEBUG_GPIO_Port GPIOA
#define THERMISTOR_1_Pin GPIO_PIN_3
#define THERMISTOR_1_GPIO_Port GPIOA
#define THERMISTOR_2_Pin GPIO_PIN_4
#define THERMISTOR_2_GPIO_Port GPIOA
#define THERMISTOR_0_Pin GPIO_PIN_5
#define THERMISTOR_0_GPIO_Port GPIOA
#define HBRIDGE_PWM_Pin GPIO_PIN_6
#define HBRIDGE_PWM_GPIO_Port GPIOA
#define HBRIDGE_BWD_Pin GPIO_PIN_7
#define HBRIDGE_BWD_GPIO_Port GPIOA
#define MOSFET_12_Pin GPIO_PIN_0
#define MOSFET_12_GPIO_Port GPIOB
#define MOSFET_11_Pin GPIO_PIN_1
#define MOSFET_11_GPIO_Port GPIOB
#define MOSFET_10_Pin GPIO_PIN_2
#define MOSFET_10_GPIO_Port GPIOB
#define MOSFET_4_Pin GPIO_PIN_10
#define MOSFET_4_GPIO_Port GPIOB
#define MOSFET_5_Pin GPIO_PIN_11
#define MOSFET_5_GPIO_Port GPIOB
#define MOSFET_9_Pin GPIO_PIN_12
#define MOSFET_9_GPIO_Port GPIOB
#define MOSFET_6_Pin GPIO_PIN_13
#define MOSFET_6_GPIO_Port GPIOB
#define SERVO_PWM_0_Pin GPIO_PIN_8
#define SERVO_PWM_0_GPIO_Port GPIOA
#define HBRIDGE_FWD_Pin GPIO_PIN_7
#define HBRIDGE_FWD_GPIO_Port GPIOC
#define LED_RED_DEBUG_Pin GPIO_PIN_10
#define LED_RED_DEBUG_GPIO_Port GPIOA
#define MOSFET_7_Pin GPIO_PIN_1
#define MOSFET_7_GPIO_Port GPIOD
#define MOSFET_8_Pin GPIO_PIN_3
#define MOSFET_8_GPIO_Port GPIOD
#define SERVO_PWM_1_Pin GPIO_PIN_3
#define SERVO_PWM_1_GPIO_Port GPIOB
#define MOSFET_2_Pin GPIO_PIN_4
#define MOSFET_2_GPIO_Port GPIOB
#define MOSFET_1_Pin GPIO_PIN_5
#define MOSFET_1_GPIO_Port GPIOB
#define SERVO_PWM_2_Pin GPIO_PIN_6
#define SERVO_PWM_2_GPIO_Port GPIOB
#define MUX_SDA_Pin GPIO_PIN_7
#define MUX_SDA_GPIO_Port GPIOB
#define MUX_SCL_Pin GPIO_PIN_8
#define MUX_SCL_GPIO_Port GPIOB
#define MOSFET_3_Pin GPIO_PIN_9
#define MOSFET_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
