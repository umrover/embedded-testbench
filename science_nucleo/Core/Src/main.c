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
#include "smbus.h"
#include "mux.h"
#include "spectral.h"
#include "thermistor.h"
//#include "mosfet.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*spectral code*/

//#define SPECTRAL_ENABLE
//#define THERMISTOR_ENABLE
//#define MOSFET_ENABLE
//#define AMMONIA_MOTOR_ENABLE
//#define PUMP_ENABLE

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* spectral code */
enum {
	SPECTRAL_0_CHANNEL = 0,
	SPECTRAL_1_CHANNEL = 1,
	SPECTRAL_2_CHANNEL = 2,
	SPECTRAL_DEVICES = 1
};

//int spectral_channels[SPECTRAL_DEVICES] = { SPECTRAL_0_CHANNEL, SPECTRAL_1_CHANNEL, SPECTRAL_2_CHANNEL };

#ifdef SPECTRAL_ENABLE
int spectral_channels[SPECTRAL_DEVICES] = { SPECTRAL_0_CHANNEL };
SMBus *i2cBus;
Spectral *spectral;

Mux *mux;
#endif
/* thermistor code */

#ifdef THERMISTOR_ENABLE
Thermistors* thermistors;
float currTemps[3];
#endif

/* mosfet code
 *
 *
 *
 */

/* ammonia motor code
 *
 *
 *
 */

/* peristaltic pump code
 *
 *
 *
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
/* spectral code */
//transmits the spectral data as a sentance
//$SPECTRAL,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,P
#ifdef SPECTRAL_ENABLE
void send_spectral_data(uint16_t *data, UART_HandleTypeDef * huart);
#endif


/* thermistor code*/

#ifdef THERMISTOR_ENABLE
#endif

/* mosfet code */
#ifdef MOSFET_ENABLE
void receive_mosfet_cmd(UART_HandleTypeDef * huart,int *device,int*enable);
#endif

/* ammonia motor code
 *
 *
 *
 */

/* peristaltic pump code
 *
 *
 *
 */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifdef SPECTRAL_ENABLE
/* spectral code */


void send_spectral_data(uint16_t *data, UART_HandleTypeDef * huart){
	int channels = 6;
	int devices = 1;

	char string[50];
	sprintf((char *)string, "$SPECTRAL");

	for (uint8_t i = 0; i < devices; ++i) {
		for (uint8_t j = 0; j < channels; ++j) {
			uint8_t msb = data[i*channels + j] >> 8;
			uint8_t lsb = data[i*channels + j];

			sprintf((char *)string + 10 + j*6,",%u,%u", msb, lsb);
		}
	}

	sprintf((char *)string + 10 + channels*6," \r\n");

	HAL_UART_Transmit(huart, (uint8_t *)string, 50, 50);
}

#endif

#ifdef THERMISTOR_ENABLE

void sendThermistorData(Thermistors* therms, UART_HandleTypeDef* huart){
  for(int t = 0; t  < 3; t++){
    currTemps[t] = getTemp(t, therms);
  }

  char *string = "";

  sprintf(string, "$THERMISTOR,%f,%f,%f", currTemps[0], currTemps[1], currTemps[2]);

  HAL_UART_Transmit(huart, (uint8_t *)string, 10, 50);

}


#endif

#ifdef MOSFET_ENABLE
  /* mosfet code */
void receive_mosfet_cmd(UART_HandleTypeDef * huart,int *device,int*enable){
  uint8_t *cmd;
  
  //Receive the bytes through Uart
  uint16_t cmdsize;
  HAL_UART_Receive(huart,cmd,cmdsize,HAL_MAX_DELAY);

  //Change to string
  char *cmdstring= "";
  // Expected $Mosfet,<devicenum>,<enablenum>
  sprintf(cmdstring,*cmd);
  char *identifier = strtok(cmdstring,"");
  device = atoi(strtok(NULL,","));
  enable = atoi(strtok(NULL,","));
}

#endif

#ifdef AMMONIA_MOTOR_ENABLE
  /* ammonia motor code
   *
   *
   *
   */
#endif

#ifdef PUMP_ENABLE
  /* peristaltic pump code
   *
   *
   *
   */
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
#ifdef SPECTRAL_ENABLE
	i2cBus = new_smbus(&hi2c3, &huart2);
	i2cBus->DMA = 0;
	mux = new_mux(i2cBus);
	spectral = new_spectral(i2cBus);
#endif

#ifdef THERMISTOR_ENABLE
  thermistors = newThermistors(&hadc2, &hadc3, &hadc4);

  sendThermistorData(thermistors, &huart2);

  deleteThermistors(thermistors);
#endif

#ifdef MOSFET_ENABLE
  /* mosfet code
   */
#endif

#ifdef AMMONIA_MOTOR_ENABLE
  /* ammonia motor code
   *
   *
   *
   */
#endif

#ifdef PUMP_ENABLE
  /* peristaltic pump code
   *
   *
   *
   */
#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

#ifdef SPECTRAL_ENABLE
	// adds all the spectral channels
	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
		add_channel(mux, spectral_channels[i]);
	}

	// opens all channels on the mux to listen

	int select = 0;
	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
		select += mux->channel_list[i];
	}
	channel_select(mux, select);

	uint8_t *buf[30];

//    strcpy((char*)buf, "success? \r\n");
//
//    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen((char*)buf), HAL_MAX_DELAY);

	enable_spectral(spectral);
	uint16_t spectral_data[SPECTRAL_DEVICES * CHANNELS];
#endif

#ifdef THERMISTOR_ENABLE
  /* thermistor code
   *
   *
   *
   */
#endif

#ifdef MOSFET_ENABLE
  /* mosfet code
   *
   *
   *
   */
#endif

#ifdef AMMONIA_MOTOR_ENABLE
  /* ammonia motor code
   *
   *
   *
   */
#endif

#ifdef PUMP_ENABLE
  /* peristaltic pump code
   *
   *
   *
   */
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef SPECTRAL_ENABLE

	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
	  channel_select(mux, mux->channel_list[spectral_channels[i]]);
	  get_spectral_data(spectral, spectral_data + (i * CHANNELS));
	}

//	uint8_t *buf[30];
//    strcpy((char*)buf, "sending data \r\n");

    //HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen((char*)buf), HAL_MAX_DELAY);
	send_spectral_data(spectral_data, &huart2);
#endif

#ifdef THERMISTOR_ENABLE
  /* thermistor code
   *
   *
   *
   */
#endif

#ifdef MOSFET_ENABLE
  int *device;
  int *enable;
  receive_mosfet_cmd(&huart2,device,enable);
  int d = *device;
  switch(d){
    case 1:
      enableRled(*enable);
      break;
    case 2:
      enableGled(*enable);
      break;
    case 3:
      enableBled(*enable);
      break;
    case 4:
      enablesciUV(*enable);
      break;
    case 5:
      enablesaUV(*enable);
      break;
    case 6:
      enableWhiteled(*enable);
      break;  
  }
#endif

#ifdef AMMONIA_MOTOR_ENABLE
  /* ammonia motor code
   *
   *
   *
   */
#endif

#ifdef PUMP_ENABLE
  /* peristaltic pump code
   *
   *
   *
   */
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_HSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x2000090E;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1080;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
