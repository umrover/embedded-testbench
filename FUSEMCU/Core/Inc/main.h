/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define CUR_IN_Pin GPIO_PIN_0
#define CUR_IN_GPIO_Port GPIOA
#define VOL_IN_Pin GPIO_PIN_1
#define VOL_IN_GPIO_Port GPIOA
#define Voltage_Select_0_Pin GPIO_PIN_2
#define Voltage_Select_0_GPIO_Port GPIOA
#define Voltage_Select_1_Pin GPIO_PIN_3
#define Voltage_Select_1_GPIO_Port GPIOA
#define Voltage_Select_2_Pin GPIO_PIN_4
#define Voltage_Select_2_GPIO_Port GPIOA
#define Voltage_Select_3_Pin GPIO_PIN_5
#define Voltage_Select_3_GPIO_Port GPIOA
#define Voltage_Enable_Pin GPIO_PIN_6
#define Voltage_Enable_GPIO_Port GPIOA
#define Current_Enable_Pin GPIO_PIN_12
#define Current_Enable_GPIO_Port GPIOB
#define Current_Select_3_Pin GPIO_PIN_13
#define Current_Select_3_GPIO_Port GPIOB
#define Current_Select_2_Pin GPIO_PIN_14
#define Current_Select_2_GPIO_Port GPIOB
#define Current_Select_1_Pin GPIO_PIN_15
#define Current_Select_1_GPIO_Port GPIOB
#define Current_Select_0_Pin GPIO_PIN_8
#define Current_Select_0_GPIO_Port GPIOA
#define JETSON_SCL_Pin GPIO_PIN_9
#define JETSON_SCL_GPIO_Port GPIOA
#define JETSON_SDA_Pin GPIO_PIN_10
#define JETSON_SDA_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
