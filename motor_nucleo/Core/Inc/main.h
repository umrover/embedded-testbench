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
#include "stm32f3xx_hal_tim.h"
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
#define CONTROL_LOOP_PERIOD 1000
#define I2C_ADDRESS 0x20
#define CHANNELS 6
#define QUADRATURE_FILTER 8
#define PWM_PERIOD 10000
#define M0_PWM_Pin GPIO_PIN_0
#define M0_PWM_GPIO_Port GPIOC
#define M1_PWM_Pin GPIO_PIN_1
#define M1_PWM_GPIO_Port GPIOC
#define M0_QUAD_A_Pin GPIO_PIN_0
#define M0_QUAD_A_GPIO_Port GPIOA
#define M0_QUAD_B_Pin GPIO_PIN_1
#define M0_QUAD_B_GPIO_Port GPIOA
#define M1_QUAD_B_Pin GPIO_PIN_4
#define M1_QUAD_B_GPIO_Port GPIOA
#define M1_QUAD_A_Pin GPIO_PIN_6
#define M1_QUAD_A_GPIO_Port GPIOA
#define M0_LIMIT_A_Pin GPIO_PIN_10
#define M0_LIMIT_A_GPIO_Port GPIOB
#define M0_LIMIT_B_Pin GPIO_PIN_11
#define M0_LIMIT_B_GPIO_Port GPIOB
#define M1_LIMIT_A_Pin GPIO_PIN_12
#define M1_LIMIT_A_GPIO_Port GPIOB
#define M1_LIMIT_B_Pin GPIO_PIN_13
#define M1_LIMIT_B_GPIO_Port GPIOB
#define M0_DIR_Pin GPIO_PIN_10
#define M0_DIR_GPIO_Port GPIOA
#define M1_DIR_Pin GPIO_PIN_11
#define M1_DIR_GPIO_Port GPIOA
#define M0_NDIR_Pin GPIO_PIN_10
#define M0_NDIR_GPIO_Port GPIOC
#define M1_NDIR_Pin GPIO_PIN_11
#define M1_NDIR_GPIO_Port GPIOC
#define J_I2C_SCL_Pin GPIO_PIN_8
#define J_I2C_SCL_GPIO_Port GPIOB
#define J_I2C_SDA_Pin GPIO_PIN_9
#define J_I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ENCODER_ERROR_THRESHOLD 0.05
#define STABILIZER_BAD_MULTIPLIER 0.97
#define STABILIZER_MULTIPLIER 0.80
#define STABILIZER_EPSILON 0.000001
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
