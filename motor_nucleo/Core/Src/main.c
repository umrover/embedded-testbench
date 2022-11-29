/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "i2c_bridge.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_MOTORS 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

Pin *hbridge_forward_pins[NUM_MOTORS] = {NULL};
Pin *hbridge_backward_pins[NUM_MOTORS] = {NULL};
HBridge *hbridges[NUM_MOTORS] = {NULL};

Pin *forward_limit_switch_pins[NUM_MOTORS] = {NULL};
Pin *backward_limit_switch_pins[NUM_MOTORS] = {NULL};

LimitSwitch *forward_limit_switches[NUM_MOTORS] = {NULL};
LimitSwitch *backward_limit_switches[NUM_MOTORS] = {NULL};

QuadEncoder *quad_encoders[NUM_MOTORS] = {NULL};

Control *controls[NUM_MOTORS] = {NULL};

Motor *motors[NUM_MOTORS] = {NULL};
I2CBus *i2c_bus;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

static void MX_TIM1_Init(void);

static void MX_TIM2_Init(void);

static void MX_TIM3_Init(void);

static void MX_TIM8_Init(void);

static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		for (size_t i = 0; i < NUM_MOTORS; ++i) {
			if (quad_encoders[i]) {
				update_quad_encoder(quad_encoders[i]);
			}
			if (forward_limit_switches[i]) {
				update_limit_switch(forward_limit_switches[i]);
			}
			if (backward_limit_switches[i]) {
				update_limit_switch(backward_limit_switches[i]);
			}
			if (motors[i]) {
				update_motor_speed(motors[i]);
			}
		}
	}
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    i2c_bus->motor_id = (0x000F & (AddrMatchCode >> 1));
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        HAL_I2C_Slave_Seq_Receive_IT(i2c_bus->i2c_bus_handle, i2c_bus->buffer, 1, I2C_LAST_FRAME);
        i2c_bus->operation = UNKNOWN;
    } else {
        // TODO - ERROR CHECKING FOR MOTOR_ID
        CH_prepare_send(i2c_bus, motors[i2c_bus->motor_id]);
        uint8_t bytes_to_send = CH_num_send(i2c_bus);
        if (bytes_to_send != 0) {
            HAL_I2C_Slave_Seq_Transmit_IT(i2c_bus->i2c_bus_handle, i2c_bus->buffer, bytes_to_send, I2C_LAST_FRAME);
        }
    }
    //HAL_IWDG_Refresh(watch_dog_handle);
    i2c_bus->tick = 0;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (i2c_bus->operation == UNKNOWN) {
        i2c_bus->operation = i2c_bus->buffer[0];
        uint8_t bytes_to_recieve = CH_num_receive(i2c_bus);
        if (bytes_to_recieve != 0) {
            HAL_I2C_Slave_Seq_Receive_IT(i2c_bus->i2c_bus_handle, i2c_bus->buffer, bytes_to_recieve, I2C_LAST_FRAME);
        } else {
            CH_process_received(i2c_bus, motors[i2c_bus->motor_id]);
        }
    } else {
        CH_process_received(i2c_bus, motors[i2c_bus->motor_id]);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    CH_reset(i2c_bus, motors, NUM_MOTORS);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(i2c_bus->i2c_bus_handle);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    hbridge_forward_pins[0] = new_pin(GPIOA, GPIO_PIN_10);
//	hbridge_forward_pins[1] = new_pin(GPIOX, GPIO_PIN_0);
//	hbridge_forward_pins[2] = new_pin(GPIOX, GPIO_PIN_0);
//	hbridge_forward_pins[3] = new_pin(GPIOX, GPIO_PIN_0);
//	hbridge_forward_pins[4] = new_pin(GPIOX, GPIO_PIN_0);
//	hbridge_forward_pins[5] = new_pin(GPIOX, GPIO_PIN_0);

    hbridge_backward_pins[0] = new_pin(GPIOC, GPIO_PIN_10);
//	hbridge_backward_pins[1] = new_pin(GPIOX, GPIO_PIN_0);
//	hbridge_backward_pins[2] = new_pin(GPIOX, GPIO_PIN_0);
//	hbridge_backward_pins[3] = new_pin(GPIOX, GPIO_PIN_0);
//	hbridge_backward_pins[4] = new_pin(GPIOX, GPIO_PIN_0);
//	hbridge_backward_pins[5] = new_pin(GPIOX, GPIO_PIN_0);

    hbridges[0] = new_hbridge(&htim1, TIM_CHANNEL_1, &(TIM1->CCR1), TIM1->ARR, hbridge_forward_pins[0],
                              hbridge_backward_pins[0]);
//	hbridges[1] = new_hbridge(&htimX, TIM_CHANNEL_X, &(TIM1->CCRX), TIMX->ARR, hbridge_forward_pins[1], hbridge_backward_pins[1]);
//	hbridges[2] = new_hbridge(&htimX, TIM_CHANNEL_X, &(TIM1->CCRX), TIMX->ARR, hbridge_forward_pins[2], hbridge_backward_pins[2]);
//	hbridges[3] = new_hbridge(&htimX, TIM_CHANNEL_X, &(TIM1->CCRX), TIMX->ARR, hbridge_forward_pins[3], hbridge_backward_pins[3]);
//	hbridges[4] = new_hbridge(&htimX, TIM_CHANNEL_X, &(TIM1->CCRX), TIMX->ARR, hbridge_forward_pins[4], hbridge_backward_pins[4]);
//	hbridges[5] = new_hbridge(&htimX, TIM_CHANNEL_X, &(TIM1->CCRX), TIMX->ARR, hbridge_forward_pins[5], hbridge_backward_pins[5]);

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
        init_hbridge(hbridges[i], 0.0f, 1);
    }

    forward_limit_switch_pins[0] = new_pin(GPIOB, GPIO_PIN_10);
//	forward_limit_switch_pins[1] = new_pin(GPIOX, GPIO_PIN_0);
//	forward_limit_switch_pins[2] = new_pin(GPIOX, GPIO_PIN_0);
//	forward_limit_switch_pins[3] = new_pin(GPIOX, GPIO_PIN_0);
//	forward_limit_switch_pins[4] = new_pin(GPIOX, GPIO_PIN_0);
//	forward_limit_switch_pins[5] = new_pin(GPIOX, GPIO_PIN_0);

    backward_limit_switch_pins[0] = new_pin(GPIOB, GPIO_PIN_11);
//	backward_limit_switch_pins[1] = new_pin(GPIOX, GPIO_PIN_0);
//	backward_limit_switch_pins[2] = new_pin(GPIOX, GPIO_PIN_0);
//	backward_limit_switch_pins[3] = new_pin(GPIOX, GPIO_PIN_0);
//	backward_limit_switch_pins[4] = new_pin(GPIOX, GPIO_PIN_0);
//	backward_limit_switch_pins[5] = new_pin(GPIOX, GPIO_PIN_0);

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
    	forward_limit_switches[i] = new_limit_switch(forward_limit_switch_pins[i]);
    	backward_limit_switches[i] = new_limit_switch(backward_limit_switch_pins[i]);
    }

    quad_encoders[0] = new_quad_encoder(&htim2, TIM2);
//	quad_encoders[1] = new_quad_encoder(&htimX, TIMX);
//	quad_encoders[2] = new_quad_encoder(&htimX, TIMX);
//	quad_encoders[3] = new_quad_encoder(&htimX, TIMX);
//	quad_encoders[4] = new_quad_encoder(&htimX, TIMX);
//	quad_encoders[5] = new_quad_encoder(&htimX, TIMX);

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
    	init_quad_encoder(quad_encoders[i]);
    }

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
        controls[i] = new_control(0.01f, 0.0f, 0.0f, 0.0f);
    }

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
        motors[i] = new_motor(
                hbridges[i],
				forward_limit_switches[i],
				backward_limit_switches[i],
                quad_encoders[i],
                controls[i]);
        init_motor(motors[i], 0.0f);
    }

    i2c_bus = new_i2c_bus(&hi2c1);

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM8_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim6);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // HAL_Delay(50);
    }

    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_TIM1
                                         | RCC_PERIPHCLK_TIM8 | RCC_PERIPHCLK_TIM2 | RCC_PERIPHCLK_TIM34;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
    PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
    PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
    PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x0000020B;
    hi2c1.Init.OwnAddress1 = 254;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
    hi2c1.Init.OwnAddress2 = 64;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_MASK04;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
        != HAL_OK) {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 199;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 30000;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
        != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
        != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
        != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
        != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = QUADRATURE_FILTER;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = QUADRATURE_FILTER;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
        != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = QUADRATURE_FILTER;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = QUADRATURE_FILTER;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
        != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 7;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = CONTROL_LOOP_PERIOD;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
        != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

    /* USER CODE BEGIN TIM8_Init 0 */

    /* USER CODE END TIM8_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM8_Init 1 */

    /* USER CODE END TIM8_Init 1 */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 7;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 1000;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
        != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM8_Init 2 */

    /* USER CODE END TIM8_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, M0_DIR_Pin | M1_DIR_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, M0_NDIR_Pin | M1_NDIR_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : M0_LIMIT_A_Pin M0_LIMIT_B_Pin M1_LIMIT_A_Pin M1_LIMIT_B_Pin */
    GPIO_InitStruct.Pin = M0_LIMIT_A_Pin | M0_LIMIT_B_Pin | M1_LIMIT_A_Pin
                          | M1_LIMIT_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : M0_DIR_Pin M1_DIR_Pin */
    GPIO_InitStruct.Pin = M0_DIR_Pin | M1_DIR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : M0_NDIR_Pin M1_NDIR_Pin */
    GPIO_InitStruct.Pin = M0_NDIR_Pin | M1_NDIR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
