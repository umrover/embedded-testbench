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
#include "command_handling.h"
//#include "abs_enc_reading.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

typedef struct{
	//REGISTERS
	uint8_t mode;
	float open_setpoint;
	int32_t closed_setpoint;
	float FF;
	float KP;
	float KI;
	float KD;

	uint8_t limit;
	float abs_enc_value;
	int32_t quad_enc_value;

	float speedMax; //max percent

	//INTERNAL
	float speed; //
	uint16_t quad_enc_raw_now;
	uint16_t quad_enc_raw_last;

	float integrated_error;
	float last_error;

	uint8_t calibrated;
	uint8_t limit_enabled;

} Channel;


extern Channel channelDefault;

extern Channel channels[6];

//reserved_area = (const char *) 0x20000088;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
float fabs(float);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CONTROL_LOOP_PERIOD 1000
#define I2C_ADDRESS 0x20
#define CHANNELS 6
#define QUADRATURE_FILTER 8
#define PWM_PERIOD 10000
#define M4_NDIR_Pin GPIO_PIN_13
#define M4_NDIR_GPIO_Port GPIOC
#define M3_NDIR_Pin GPIO_PIN_14
#define M3_NDIR_GPIO_Port GPIOC
#define M5_NDIR_Pin GPIO_PIN_15
#define M5_NDIR_GPIO_Port GPIOC
#define M_I2C_SDA_Pin GPIO_PIN_0
#define M_I2C_SDA_GPIO_Port GPIOF
#define M_I2C_SCL_Pin GPIO_PIN_1
#define M_I2C_SCL_GPIO_Port GPIOF
#define M0_PWM_Pin GPIO_PIN_0
#define M0_PWM_GPIO_Port GPIOC
#define M1_PWM_Pin GPIO_PIN_1
#define M1_PWM_GPIO_Port GPIOC
#define M2_PWM_Pin GPIO_PIN_2
#define M2_PWM_GPIO_Port GPIOC
#define M0_QUAD_A_Pin GPIO_PIN_0
#define M0_QUAD_A_GPIO_Port GPIOA
#define M0_QUAD_B_Pin GPIO_PIN_1
#define M0_QUAD_B_GPIO_Port GPIOA
#define M1_QUAD_B_Pin GPIO_PIN_4
#define M1_QUAD_B_GPIO_Port GPIOA
#define M1_QUAD_A_Pin GPIO_PIN_6
#define M1_QUAD_A_GPIO_Port GPIOA
#define M3_DIR_Pin GPIO_PIN_2
#define M3_DIR_GPIO_Port GPIOB
#define M0_LIMIT_Pin GPIO_PIN_10
#define M0_LIMIT_GPIO_Port GPIOB
#define M1_LIMIT_Pin GPIO_PIN_11
#define M1_LIMIT_GPIO_Port GPIOB
#define M2_LIMIT_Pin GPIO_PIN_12
#define M2_LIMIT_GPIO_Port GPIOB
#define M3_LIMIT_Pin GPIO_PIN_13
#define M3_LIMIT_GPIO_Port GPIOB
#define M4_LIMIT_Pin GPIO_PIN_14
#define M4_LIMIT_GPIO_Port GPIOB
#define M5_LIMIT_Pin GPIO_PIN_15
#define M5_LIMIT_GPIO_Port GPIOB
#define M3_PWM_Pin GPIO_PIN_6
#define M3_PWM_GPIO_Port GPIOC
#define M4_PWM_Pin GPIO_PIN_7
#define M4_PWM_GPIO_Port GPIOC
#define M5_PWM_Pin GPIO_PIN_8
#define M5_PWM_GPIO_Port GPIOC
#define M0_DIR_Pin GPIO_PIN_10
#define M0_DIR_GPIO_Port GPIOA
#define M1_DIR_Pin GPIO_PIN_11
#define M1_DIR_GPIO_Port GPIOA
#define M2_DIR_Pin GPIO_PIN_12
#define M2_DIR_GPIO_Port GPIOA
#define M5_DIR_Pin GPIO_PIN_15
#define M5_DIR_GPIO_Port GPIOA
#define M0_NDIR_Pin GPIO_PIN_10
#define M0_NDIR_GPIO_Port GPIOC
#define M1_NDIR_Pin GPIO_PIN_11
#define M1_NDIR_GPIO_Port GPIOC
#define M2_NDIR_Pin GPIO_PIN_12
#define M2_NDIR_GPIO_Port GPIOC
#define M4_DIR_Pin GPIO_PIN_6
#define M4_DIR_GPIO_Port GPIOB
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
