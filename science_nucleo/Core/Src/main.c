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
#include "mosfet.h"
#include "ammonia_motor.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//#define SPECTRAL_ENABLE
//#define THERMISTOR_ENABLE//
#define MOSFET_ENABLE
//#define AMMONIA_MOTOR_ENABLE

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* spectral code */

#ifdef SPECTRAL_ENABLE
enum {
	SPECTRAL_0_CHANNEL = 0,
	SPECTRAL_1_CHANNEL = 1,
	SPECTRAL_2_CHANNEL = 2,
	SPECTRAL_DEVICES = 1
};

//int spectral_channels[SPECTRAL_DEVICES] = { SPECTRAL_0_CHANNEL, SPECTRAL_1_CHANNEL, SPECTRAL_2_CHANNEL };
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

/* ammonia motor code */

#ifdef AMMONIA_MOTOR_ENABLE
Motor *ammonia_motor;
#endif

/* peristaltic pump code
 *
 *
 *
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// Clears the ORE and NE flags from the uart handler with delay
void clear_flags();


/* spectral code */
//transmits the spectral data as a sentance
//$SPECTRAL,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,
#ifdef SPECTRAL_ENABLE
void send_spectral_data(uint16_t *data, UART_HandleTypeDef * huart);
#endif


/* thermistor code*/

#ifdef THERMISTOR_ENABLE
#endif

/* mosfet code */
#ifdef MOSFET_ENABLE
void receive_mosfet_cmd(uint8_t *buffer,int *device, int*enable);
#endif

/* ammonia motor code */
#ifdef AMMONIA_MOTOR_ENABLE
void receive_ammonia_motor_cmd(uint8_t *buffer, double *speed);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Clears the ORE and NE flags from the uart handler with delay
void clear_flags(){
	HAL_Delay(250);
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	__HAL_UART_CLEAR_NEFLAG(&huart1);
}

#ifdef SPECTRAL_ENABLE
/* spectral code */


void send_spectral_data(uint16_t *data, UART_HandleTypeDef * huart){
	int channels = 6;
	int devices = 1;

	char string[50];
	sprintf((char *)string, "$SPECTRAL");

	for (uint8_t i = 0; i < devices; ++i) {
		uint8_t start = i * (channels*6) + 10;
		for (uint8_t j = 0; j < channels; ++j) {
			uint8_t msb = data[i*channels + j] >> 8;
			uint8_t lsb = data[i*channels + j];

			sprintf((char *)string + start + j*6,",%u,%u", msb, lsb);
		}
	}

	sprintf((char *)string + 10 + channels*6," \r\n");

	// testing stuff - use if you don't have a
	//char test[120];
	//sprintf((char *)test, "$SPECTRAL,3,4,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,\n");

	HAL_UART_Transmit(huart, (uint8_t *)string, 50, 50);
	HAL_Delay(100);


}

#endif

#ifdef THERMISTOR_ENABLE

void sendThermistorData(Thermistors* therms, UART_HandleTypeDef* huart){
  for(int t = 0; t < 3; t++){
    currTemps[t] = getTemp(t, therms);
  }

  char string[50] = "";

  sprintf((char *)string, "$THERMISTOR,%f,%f,%f,\n", currTemps[0], currTemps[1], currTemps[2]);
  HAL_UART_Transmit(huart, (uint8_t *)string, sizeof(string), 50);
  // Delay before Clearing flags so beaglebone can successfully read the 
  HAL_Delay(100);


}


#endif

#ifdef MOSFET_ENABLE
  /* mosfet code */
void receive_mosfet_cmd(uint8_t *buffer, int *device,int*enable){

  //Change to string
  char delim[] = ",";
  char *copy = (char *)malloc(strlen(buffer) + 1);
  if (copy == NULL) {
  	return;
  }
  strcpy(copy, buffer);
  //Expected $Mosfet,<devicenum>,<enablenum>
  char *identifier = strtok(copy,delim);
  if (!strcmp(identifier,"$Mosfet")){
	  *device = atoi(strtok(NULL,delim));
	  *enable = atoi(strtok(NULL,delim));
  }
}

void send_rr_drop(UART_HandleTypeDef* huart){
	char string[10];
	sprintf((char *)string, "$REPEATER\n");
	HAL_UART_Transmit(huart, (uint8_t *)string, sizeof(string), 11);
}


#endif

#ifdef AMMONIA_MOTOR_ENABLE
  /* ammonia motor code */

void receive_ammonia_motor_cmd(uint8_t *buffer, double *speed) {
	// expects $AMMONIA, <speed>, <comma padding>
	char delim[] = ",";
	char *copy = (char *)malloc(strlen(buffer) + 1);
	if (copy == NULL) {
	  return;
	}
	strcpy(copy, buffer);
	const char *identifier =  strtok(copy, delim);
	if (!strcmp(identifier, "$AMMONIA")) {
		*speed = atof(strtok(NULL, delim));
	}
}
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
 h  HAL_Init();

  /* USER CODE BEGIN Init */
#ifdef SPECTRAL_ENABLE
	i2cBus = new_smbus(&hi2c2, &huart2);
	i2cBus->DMA = 0;
	mux = new_mux(i2cBus);
	spectral = new_spectral(i2cBus);
#endif

#ifdef THERMISTOR_ENABLE
  thermistors = newThermistors(&hadc2, &hadc3, &hadc4);

  

#endif

#ifdef MOSFET_ENABLE
  /* mosfet code
   */
#endif

#ifdef AMMONIA_MOTOR_ENABLE
  // pin B9 is only being used here because one of the GPIO pins on the testing
  // nucleo broke --> for SAR it will be pin C14
  ammonia_motor = new_motor(GPIOC, GPIO_PIN_2, GPIOB, GPIO_PIN_10, &htim3);
#endif

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
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

	enablesciUV(0);
	enablesaUV(0);
	enableWhiteled(0);

#endif

#ifdef AMMONIA_MOTOR_ENABLE
  /* ammonia motor code */
	start(ammonia_motor, TIM_CHANNEL_1);
	double speed = 0.0;
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t Rx_data[20];
  while (1)
  {
	// receive the UART data string
	for(int i = 0; i< 20; ++i){
		Rx_data[i] = 0;
	}
	// Jank fix to stop readline from blocking in science_bridge
	// Only use if thermistor/spectral is not sending
	char emp[7];
	sprintf((char *)emp, "$EMPTY\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)emp, sizeof(emp), 11);

	HAL_UART_Receive(&huart1, Rx_data, 13, 23000);
	clear_flags();



    // Read and send all thermistor data over huart1
#ifdef THERMISTOR_ENABLE
    sendThermistorData(thermistors, &huart1);
    clear_flags();
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef SPECTRAL_ENABLE

	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
	  channel_select(mux, mux->channel_list[spectral_channels[i]]);
	  get_spectral_data(spectral, spectral_data + (i * CHANNELS));
	}

	send_spectral_data(spectral_data, &huart1);
    clear_flags();
#endif

#ifdef MOSFET_ENABLE
	int device = 8;
	int enable = 0;
	receive_mosfet_cmd(Rx_data,&device,&enable);

	int d = device;
	switch(d){
	case 0 :
	  //enableRled(enable);
	  enablePin(enable, Auton_Red_LED_GPIO_Port, Auton_Red_LED_Pin);
	  break;
	case 1 :
	  //enableGled(enable);
	  enablePin(enable, Auton_Green_LED_GPIO_Port, Auton_Green_LED_Pin);
	  break;
	case 2:
	  //enableBled(enable);
	  enablePin(enable, Auton_Blue_LED_GPIO_Port, Auton_Blue_LED_Pin);
	  break;
	case 3:
	  //enablesciUV(enable);
	  enablePin(enable, sci_UV_LED_GPIO_Port, sci_UV_LED_Pin);
	  break;
	case 4:
	  //enablesaUV(enable);
	  enablePin(enable, SA_UV_LED_GPIO_Port, SA_UV_LED_Pin);
	  send_rr_drop(&huart1);
	  break;
	case 5:
	  enablePin(enable, whiteLED_GPIO_Port, whiteLED_Pin);
	  //enableWhiteled(enable);
	  break;
	case 6:
	  //enablePerPump0(enable);
	  enablePin(enable, Pump_1_GPIO_Port, Pump_1_Pin);
	  break;
 	case 7:
 	  //enablePerPump1(enable);
 	  enablePin(enable, Pump_2_GPIO_Port, Pump_2_Pin);
 	  break;
 	case 8:
 	  break;
 	}






#endif

#ifdef AMMONIA_MOTOR_ENABLE
  /* ammonia motor code */
	receive_ammonia_motor_cmd(Rx_data, &speed);
	set_speed(ammonia_motor, speed);

#endif


  }
#ifdef THERMISTOR_ENABLE
  deleteThermistors(thermistors);
#endif
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC34|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */
  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Ammonia_FWD_Pin|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_2
                          |Auton_Green_LED_Pin|Auton_Blue_LED_Pin|Auton_Red_LED_Pin|sci_UV_LED_Pin
                          |SA_UV_LED_Pin|Pump_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Ammonia_BWD_Pin|Pump_1_Pin|whiteLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Ammonia_FWD_Pin PC15 PC0 PC2
                           Auton_Green_LED_Pin Auton_Blue_LED_Pin Auton_Red_LED_Pin sci_UV_LED_Pin
                           SA_UV_LED_Pin Pump_2_Pin */
  GPIO_InitStruct.Pin = Ammonia_FWD_Pin|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_2
                          |Auton_Green_LED_Pin|Auton_Blue_LED_Pin|Auton_Red_LED_Pin|sci_UV_LED_Pin
                          |SA_UV_LED_Pin|Pump_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Ammonia_BWD_Pin Pump_1_Pin whiteLED_Pin */
  GPIO_InitStruct.Pin = Ammonia_BWD_Pin|Pump_1_Pin|whiteLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
