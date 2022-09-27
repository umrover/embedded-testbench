/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define LIMIT_SWITCH_0_Pin GPIO_PIN_0
#define LIMIT_SWITCH_0_GPIO_Port GPIOC
#define LIMIT_SWITCH_1_Pin GPIO_PIN_1
#define LIMIT_SWITCH_1_GPIO_Port GPIOC
#define LIMIT_SWITCH_2_Pin GPIO_PIN_2
#define LIMIT_SWITCH_2_GPIO_Port GPIOC
#define LIMIT_SWITCH_3_Pin GPIO_PIN_3
#define LIMIT_SWITCH_3_GPIO_Port GPIOC
#define MOTOR_4_PWM_Pin GPIO_PIN_0
#define MOTOR_4_PWM_GPIO_Port GPIOA
#define MOTOR_5_PWM_Pin GPIO_PIN_1
#define MOTOR_5_PWM_GPIO_Port GPIOA
#define MOTOR_6_PWM_Pin GPIO_PIN_2
#define MOTOR_6_PWM_GPIO_Port GPIOA
#define MOTOR_7_PWM_Pin GPIO_PIN_3
#define MOTOR_7_PWM_GPIO_Port GPIOA
#define MOTOR_4_DIR_Pin GPIO_PIN_4
#define MOTOR_4_DIR_GPIO_Port GPIOA
#define MOTOR_4_NDIR_Pin GPIO_PIN_5
#define MOTOR_4_NDIR_GPIO_Port GPIOA
#define MOTOR_5_DIR_Pin GPIO_PIN_6
#define MOTOR_5_DIR_GPIO_Port GPIOA
#define MOTOR_5_NDIR_Pin GPIO_PIN_7
#define MOTOR_5_NDIR_GPIO_Port GPIOA
#define MOTOR_6_DIR_Pin GPIO_PIN_4
#define MOTOR_6_DIR_GPIO_Port GPIOC
#define MOTOR_6_NDIR_Pin GPIO_PIN_5
#define MOTOR_6_NDIR_GPIO_Port GPIOC
#define MOTOR_7_DIR_Pin GPIO_PIN_0
#define MOTOR_7_DIR_GPIO_Port GPIOB
#define MOTOR_7_NDIR_Pin GPIO_PIN_1
#define MOTOR_7_NDIR_GPIO_Port GPIOB
#define LIMIT_SWITCH_4_Pin GPIO_PIN_2
#define LIMIT_SWITCH_4_GPIO_Port GPIOB
#define LIMIT_SWITCH_5_Pin GPIO_PIN_7
#define LIMIT_SWITCH_5_GPIO_Port GPIOE
#define LIMIT_SWITCH_6_Pin GPIO_PIN_8
#define LIMIT_SWITCH_6_GPIO_Port GPIOE
#define LIMIT_SWITCH_7_Pin GPIO_PIN_9
#define LIMIT_SWITCH_7_GPIO_Port GPIOE
#define MOTOR_12_DIR_Pin GPIO_PIN_10
#define MOTOR_12_DIR_GPIO_Port GPIOE
#define MOTOR_12_NDIR_Pin GPIO_PIN_11
#define MOTOR_12_NDIR_GPIO_Port GPIOE
#define MOTOR_13_DIR_Pin GPIO_PIN_12
#define MOTOR_13_DIR_GPIO_Port GPIOE
#define MOTOR_13_NDIR_Pin GPIO_PIN_13
#define MOTOR_13_NDIR_GPIO_Port GPIOE
#define MOTOR_10_DIR_Pin GPIO_PIN_14
#define MOTOR_10_DIR_GPIO_Port GPIOE
#define MOTOR_10_NDIR_Pin GPIO_PIN_15
#define MOTOR_10_NDIR_GPIO_Port GPIOE
#define I2C_SCL_BACKUP_Pin GPIO_PIN_10
#define I2C_SCL_BACKUP_GPIO_Port GPIOB
#define I2C_SDA_BACKUP_Pin GPIO_PIN_11
#define I2C_SDA_BACKUP_GPIO_Port GPIOB
#define MOTOR_11_DIR_Pin GPIO_PIN_12
#define MOTOR_11_DIR_GPIO_Port GPIOB
#define MOTOR_11_NDIR_Pin GPIO_PIN_13
#define MOTOR_11_NDIR_GPIO_Port GPIOB
#define MOTOR_12_PWM_Pin GPIO_PIN_14
#define MOTOR_12_PWM_GPIO_Port GPIOB
#define MOTOR_13_PWM_Pin GPIO_PIN_15
#define MOTOR_13_PWM_GPIO_Port GPIOB
#define MOTOR_8_DIR_Pin GPIO_PIN_8
#define MOTOR_8_DIR_GPIO_Port GPIOD
#define MOTOR_8_NDIR_Pin GPIO_PIN_9
#define MOTOR_8_NDIR_GPIO_Port GPIOD
#define MOTOR_9_DIR_Pin GPIO_PIN_10
#define MOTOR_9_DIR_GPIO_Port GPIOD
#define MOTOR_9_NDIR_Pin GPIO_PIN_11
#define MOTOR_9_NDIR_GPIO_Port GPIOD
#define QUAD_1_A_Pin GPIO_PIN_12
#define QUAD_1_A_GPIO_Port GPIOD
#define QUAD_1_B_Pin GPIO_PIN_13
#define QUAD_1_B_GPIO_Port GPIOD
#define MOTOR_10_PWM_Pin GPIO_PIN_14
#define MOTOR_10_PWM_GPIO_Port GPIOD
#define MOTOR_11_PWM_Pin GPIO_PIN_15
#define MOTOR_11_PWM_GPIO_Port GPIOD
#define QUAD_0_A_Pin GPIO_PIN_6
#define QUAD_0_A_GPIO_Port GPIOC
#define QUAD_0_B_Pin GPIO_PIN_7
#define QUAD_0_B_GPIO_Port GPIOC
#define MOTOR_8_PWM_Pin GPIO_PIN_8
#define MOTOR_8_PWM_GPIO_Port GPIOC
#define MOTOR_9_PWM_Pin GPIO_PIN_9
#define MOTOR_9_PWM_GPIO_Port GPIOC
#define MOTOR_0_PWM_Pin GPIO_PIN_8
#define MOTOR_0_PWM_GPIO_Port GPIOA
#define MOTOR_1_PWM_Pin GPIO_PIN_9
#define MOTOR_1_PWM_GPIO_Port GPIOA
#define MOTOR_2_PWM_Pin GPIO_PIN_10
#define MOTOR_2_PWM_GPIO_Port GPIOA
#define MOTOR_3_PWM_Pin GPIO_PIN_11
#define MOTOR_3_PWM_GPIO_Port GPIOA
#define MOTOR_0_DIR_Pin GPIO_PIN_10
#define MOTOR_0_DIR_GPIO_Port GPIOC
#define MOTOR_0_NDIR_Pin GPIO_PIN_11
#define MOTOR_0_NDIR_GPIO_Port GPIOC
#define MOTOR_1_DIR_Pin GPIO_PIN_12
#define MOTOR_1_DIR_GPIO_Port GPIOC
#define MOTOR_1_NDIR_Pin GPIO_PIN_0
#define MOTOR_1_NDIR_GPIO_Port GPIOD
#define MOTOR_2_DIR_Pin GPIO_PIN_1
#define MOTOR_2_DIR_GPIO_Port GPIOD
#define MOTOR_2_NDIR_Pin GPIO_PIN_2
#define MOTOR_2_NDIR_GPIO_Port GPIOD
#define MOTOR_3_DIR_Pin GPIO_PIN_3
#define MOTOR_3_DIR_GPIO_Port GPIOD
#define MOTOR_3_NDIR_Pin GPIO_PIN_4
#define MOTOR_3_NDIR_GPIO_Port GPIOD
#define USART_BACKUP_TX_1_Pin GPIO_PIN_5
#define USART_BACKUP_TX_1_GPIO_Port GPIOD
#define USART_BACKUP_RX_1_Pin GPIO_PIN_6
#define USART_BACKUP_RX_1_GPIO_Port GPIOD
#define USART_TX_BACKUP_0_Pin GPIO_PIN_6
#define USART_TX_BACKUP_0_GPIO_Port GPIOB
#define USART_RX_BACKUP_0_Pin GPIO_PIN_7
#define USART_RX_BACKUP_0_GPIO_Port GPIOB
#define I2C_SCL_TO_JETSON_Pin GPIO_PIN_8
#define I2C_SCL_TO_JETSON_GPIO_Port GPIOB
#define I2C_SDA_TO_JETSON_Pin GPIO_PIN_9
#define I2C_SDA_TO_JETSON_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
