/* USER CODE BEGIN Header */


//stack_size for MiscTaskHandle and SpectralTaskHandle should be set to 128 * 10. The ioc might overwrite it.
//Make sure to change the #include's for the .h files under Core/Inc to the correct STM when changing STM types.
//(i.e #include "stm32g0xx_hal.h" ----> #include "stm32f1xx_hal.h")
//Spectral code in StartSpectralTask() is buggy, so it is commented out for now.


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_sensor.h"
#include "bridge.h"
#include "diag_curr_sensor.h"
#include "diag_temp_sensor.h"
#include "heater.h"
#include "pin_data.h"
#include "servo.h"
#include "smbus.h"
#include "spectral.h"
#include "thermistor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_MOSFET_DEVICES 12
#define NUM_DIAG_CURRENT_SENSORS 3
#define NUM_DIAG_TEMP_SENSORS 3
#define NUM_SCIENCE_TEMP_SENSORS 3
#define NUM_SERVOS 3
#define NUM_HEATERS 3
#define NUM_DEBUG_LEDS 4

#define HEATER_0_MOSFET_PIN 3
#define HEATER_1_MOSFET_PIN 4
#define HEATER_2_MOSFET_PIN 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for MiscTask */
osThreadId_t MiscTaskHandle;
const osThreadAttr_t MiscTask_attributes = {
  .name = "MiscTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 10
};
/* Definitions for SpectralTask */
osThreadId_t SpectralTaskHandle;
const osThreadAttr_t SpectralTask_attributes = {
  .name = "SpectralTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 10
};
/* USER CODE BEGIN PV */
ADCSensor* adc_sensor = NULL;
Bridge* bridge = NULL;
DiagCurrentSensor* diag_current_sensors[NUM_DIAG_CURRENT_SENSORS] = {NULL};
DiagTempSensor* diag_temp_sensors[NUM_DIAG_TEMP_SENSORS] = {NULL};
Heater* science_heaters[NUM_HEATERS] = {NULL};
PinData* debug_leds[NUM_DEBUG_LEDS] = {NULL};
PinData* heater_pins[NUM_HEATERS] = {NULL};
PinData* mosfet_pins[NUM_MOSFET_DEVICES] = {NULL};
Servo* servos[NUM_SERVOS] = {NULL};
SMBus* smbus = NULL;
Spectral* spectral = NULL;
Thermistor* science_temp_sensors[NUM_SCIENCE_TEMP_SENSORS] = {NULL};


float diag_temperatures[NUM_DIAG_TEMP_SENSORS] = {0};
float diag_currents[NUM_DIAG_CURRENT_SENSORS] = {0};
float science_temperatures[NUM_SCIENCE_TEMP_SENSORS] = {0};
uint16_t spectral_data[SPECTRAL_CHANNELS] = {0};
bool heater_auto_shutoff_state = false;
bool heater_on_state[NUM_HEATERS] = {false};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void StartMiscTask(void *argument);
void StartSpectralTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	receive_bridge(bridge, science_heaters, mosfet_pins, servos);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	adc_sensor = new_adc_sensor(&hadc1, 9);

		bridge = new_bridge(&huart1);

		diag_current_sensors[0] = new_diag_current_sensor(adc_sensor, 3);
		diag_current_sensors[1] = new_diag_current_sensor(adc_sensor, 4);
		diag_current_sensors[2] = new_diag_current_sensor(adc_sensor, 5);

		diag_temp_sensors[0] = new_diag_temp_sensor(adc_sensor, 6);
		diag_temp_sensors[1] = new_diag_temp_sensor(adc_sensor, 7);
		diag_temp_sensors[2] = new_diag_temp_sensor(adc_sensor, 8);

		debug_leds[0] = new_pin_data(DEBUG_LED_0_GPIO_Port, DEBUG_LED_0_Pin, PIN_IS_OUTPUT);
		debug_leds[1] = new_pin_data(DEBUG_LED_1_GPIO_Port, DEBUG_LED_1_Pin, PIN_IS_OUTPUT);
		debug_leds[2] = new_pin_data(DEBUG_LED_2_GPIO_Port, DEBUG_LED_2_Pin, PIN_IS_OUTPUT);
		debug_leds[3] = new_pin_data(DEBUG_LED_3_GPIO_Port, DEBUG_LED_3_Pin, PIN_IS_OUTPUT);

		mosfet_pins[0] = new_pin_data(MOSFET_0_GPIO_Port, MOSFET_0_Pin, PIN_IS_OUTPUT);
		mosfet_pins[1] = new_pin_data(MOSFET_1_GPIO_Port, MOSFET_1_Pin, PIN_IS_OUTPUT);
		mosfet_pins[2] = new_pin_data(MOSFET_2_GPIO_Port, MOSFET_2_Pin, PIN_IS_OUTPUT);
		mosfet_pins[3] = new_pin_data(MOSFET_3_GPIO_Port, MOSFET_3_Pin, PIN_IS_OUTPUT);
		mosfet_pins[4] = new_pin_data(MOSFET_4_GPIO_Port, MOSFET_4_Pin, PIN_IS_OUTPUT);
		mosfet_pins[5] = new_pin_data(MOSFET_5_GPIO_Port, MOSFET_5_Pin, PIN_IS_OUTPUT);
		mosfet_pins[6] = new_pin_data(MOSFET_6_GPIO_Port, MOSFET_6_Pin, PIN_IS_OUTPUT);
		mosfet_pins[7] = new_pin_data(MOSFET_7_GPIO_Port, MOSFET_7_Pin, PIN_IS_OUTPUT);
		mosfet_pins[8] = new_pin_data(MOSFET_8_GPIO_Port, MOSFET_8_Pin, PIN_IS_OUTPUT);
		mosfet_pins[9] = new_pin_data(MOSFET_9_GPIO_Port, MOSFET_9_Pin, PIN_IS_OUTPUT);
		mosfet_pins[10] = new_pin_data(MOSFET_10_GPIO_Port, MOSFET_10_Pin, PIN_IS_OUTPUT);
		mosfet_pins[11] = new_pin_data(MOSFET_11_GPIO_Port, MOSFET_11_Pin, PIN_IS_OUTPUT);

		heater_pins[0] = mosfet_pins[HEATER_0_MOSFET_PIN];
		heater_pins[1] = mosfet_pins[HEATER_1_MOSFET_PIN];
		heater_pins[2] = mosfet_pins[HEATER_2_MOSFET_PIN];

		servos[0] = new_servo(&htim1, TIM_CHANNEL_1, &(TIM1->CCR1));
		servos[1] = new_servo(&htim1, TIM_CHANNEL_2, &(TIM1->CCR2));
		servos[2] = new_servo(&htim1, TIM_CHANNEL_3, &(TIM1->CCR3));

		smbus = new_smbus(&hi2c1, NULL, false);
		spectral = new_spectral(smbus);

		science_temp_sensors[0] = new_thermistor(adc_sensor, 0);
		science_temp_sensors[1] = new_thermistor(adc_sensor, 1);
		science_temp_sensors[2] = new_thermistor(adc_sensor, 2);

		science_heaters[0] = new_heater(heater_pins[0], science_temp_sensors[0]);
		science_heaters[1] = new_heater(heater_pins[1], science_temp_sensors[1]);
		science_heaters[2] = new_heater(heater_pins[2], science_temp_sensors[2]);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  receive_bridge(bridge, science_heaters, mosfet_pins, servos);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MiscTask */
  MiscTaskHandle = osThreadNew(StartMiscTask, NULL, &MiscTask_attributes);

  /* creation of SpectralTask */
  SpectralTaskHandle = osThreadNew(StartSpectralTask, NULL, &SpectralTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOSFET_0_Pin|MOSFET_1_Pin|MOSFET_2_Pin|MOSFET_3_Pin
                          |MOSFET_4_Pin|MOSFET_5_Pin|DEBUG_LED_1_Pin|DEBUG_LED_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOSFET_6_Pin|MOSFET_11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOSFET_7_Pin|MOSFET_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MOSFET_9_Pin|MOSFET_10_Pin|DEBUG_LED_3_Pin|DEBUG_LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOSFET_0_Pin MOSFET_1_Pin MOSFET_2_Pin MOSFET_3_Pin
                           MOSFET_4_Pin MOSFET_5_Pin DEBUG_LED_1_Pin DEBUG_LED_0_Pin */
  GPIO_InitStruct.Pin = MOSFET_0_Pin|MOSFET_1_Pin|MOSFET_2_Pin|MOSFET_3_Pin
                          |MOSFET_4_Pin|MOSFET_5_Pin|DEBUG_LED_1_Pin|DEBUG_LED_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOSFET_6_Pin MOSFET_11_Pin */
  GPIO_InitStruct.Pin = MOSFET_6_Pin|MOSFET_11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOSFET_7_Pin MOSFET_8_Pin */
  GPIO_InitStruct.Pin = MOSFET_7_Pin|MOSFET_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOSFET_9_Pin MOSFET_10_Pin DEBUG_LED_3_Pin DEBUG_LED_2_Pin */
  GPIO_InitStruct.Pin = MOSFET_9_Pin|MOSFET_10_Pin|DEBUG_LED_3_Pin|DEBUG_LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMiscTask */
/**
  * @brief  Function implementing the MiscTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMiscTask */
void StartMiscTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime;
	 const TickType_t xFrequency = 5000/portTICK_PERIOD_MS; //values in milliseconds
	 xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  	  /*
	  	   * In a while loop, we will need to get the following data and send it over:
	  	   * Diagnostic Temperatures and Currents
	  	   * Science Temperatures
	  	   * State of auto-shut off state of the heater (upon change).
	  	   * State of the heater (upon change).
	  	   */

	    	  update_adc_sensor_values(adc_sensor);
	  	  for (size_t i = 0; i < NUM_DIAG_TEMP_SENSORS; ++i) {
	  		  update_diag_temp_sensor_val(diag_temp_sensors[i]);
	  		  diag_temperatures[i] = get_diag_temp_sensor_val(diag_temp_sensors[i]);
	  	  }
	  	  for (size_t i = 0; i < NUM_DIAG_CURRENT_SENSORS; ++i) {
	  		  update_diag_current_sensor_val(diag_current_sensors[i]);
	  		  diag_currents[i] = get_diag_current_sensor_val(diag_current_sensors[i]);
	  	  }
	  	  bridge_send_diagnostic(bridge, diag_temperatures, diag_currents);

	  	  for (size_t i = 0; i < NUM_HEATERS; ++i) {
	  		  update_heater_temperature(science_heaters[i]);
	  		  update_heater_state(science_heaters[i]);
	  		  science_temperatures[i] = get_thermistor_temperature(science_temp_sensors[i]);
	  	  }
	  	  bridge_send_science_thermistors(bridge, science_temperatures);

	    	  bool send_auto_shutoff = false;
	  	  for (size_t i = 0; i < NUM_HEATERS; ++i) {
	  		  heater_auto_shutoff_state = science_heaters[i]->auto_shutoff;
	  		  if (science_heaters[i]->send_auto_shutoff) {
	  			  science_heaters[i]->send_auto_shutoff = false;
	  			  send_auto_shutoff = true;
	  		  }
	  	  }
	  	  if (send_auto_shutoff) {
	  		  bridge_send_heater_auto_shutoff(bridge, heater_auto_shutoff_state);
	  	  }

	  	  bool send_heater_on = false;
	  	  for (size_t i = 0; i < NUM_HEATERS; ++i) {
	  		  heater_on_state[i] = science_heaters[i]->is_on;
	  		  if (science_heaters[i]->send_on) {
	  			  science_heaters[i]->send_on = false;
	  			  send_heater_on = true;
	  		  }
	  	  }
	  	  if (send_heater_on) {
	  		  bridge_send_heater_state(bridge, heater_on_state);
	  	  }
	  	vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  osThreadTerminate(NULL); //May want this just in case
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSpectralTask */
/**
* @brief Function implementing the SpectralTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSpectralTask */
void StartSpectralTask(void *argument)
{
  /* USER CODE BEGIN StartSpectralTask */
	TickType_t xLastWakeTime;
		 const TickType_t xFrequency = 5000/portTICK_PERIOD_MS; //value in milliseconds
		 xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
  	  /*
	   * In a while loop, we will need to get the following data and send it over:
	   * Spectral sensor data
	   */

	  /*
  	  for (size_t i = 0; i < SPECTRAL_CHANNELS; ++i) {
		  update_spectral_channel_data(spectral, i);
		  spectral_data[i] = get_spectral_channel_data(spectral, i);
	  }
	  bridge_send_spectral(bridge, spectral_data);
	  */
	  vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  osThreadTerminate(NULL); // May want this just in case
  /* USER CODE END StartSpectralTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
