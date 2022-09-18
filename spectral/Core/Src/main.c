/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TODO 0

// Spectral Device - AS7263
// DATASHEET - https://cdn.sparkfun.com/assets/1/b/7/3/b/AS7263.pdf

// TODO - Find the I2C device address
// Hint - check page 18
#define I2C_DEV_ADDRESS ((uint8_t)TODO)

// TODO - Find the control set up register
// Hint - check page 21
#define CONTROL_SET_UP_REGISTER TODO


// TODO - Find the int time register
// Hint - check page 21
#define INTEGRATION_TIME_REGISTER TODO

// TODO - Find the write and read registers
// Hint - check page 18
#define CHANNEL_WRITE_REGISTER ((uint8_t)TODO)
#define CHANNEL_READ_REGISTER ((uint8_t)TODO)

// TODO - Find out what value to put into CHANNEL_WRITE_REGISTER
// if you want to read the MSB from channel 0, which we
// will call the start channel value.
// The start channel is channel R, and the available channels are RSTUVW.
// Hint - check page 21
#define START_CHANNEL_VAL ((uint8_t)TODO)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TODO - Read through this function to see how to read a byte via I2C
uint8_t read_byte_data(uint8_t addr, char cmd) {

	uint8_t buf[30];
	HAL_StatusTypeDef ret;
	I2C_HandleTypeDef *i2c = &hi2c1;

	// First we need to initiate the transaction by having the Nucleo send a one byte message
	// the contains the command.
    buf[0] = cmd;
    ret = HAL_I2C_Master_Transmit(i2c, addr << 1, buf, 1, 50);

    // After sending in the command, the Nucleo should read in the 1 byte response.
    ret = HAL_I2C_Master_Receive(i2c, addr << 1 | 1, buf, 1, 50);

    if (ret != HAL_OK)
    {
    	HAL_I2C_DeInit(i2c);
    	HAL_Delay(5);
    	HAL_I2C_Init(i2c);
      return -1;
    }

    return buf[0];
}

// TODO - Read through this function to see how to write a byte via I2C
void write_byte_data(uint8_t addr, char cmd, uint8_t data) {
	uint8_t buf[30];
	HAL_StatusTypeDef ret;
	I2C_HandleTypeDef *i2c = &hi2c1;
    buf[0] = data;
    buf[1] = cmd;
    ret = HAL_I2C_Master_Transmit(i2c, addr << 1, buf, 2, 50);

    if (ret != HAL_OK)
	{
		HAL_I2C_DeInit(i2c);
		HAL_Delay(5);
		HAL_I2C_Init(i2c);
		data = 0;
	}
}

// TODO - implement this function
uint8_t get_value(uint8_t channel_val) {
	write_byte_data(I2C_DEV_ADDRESS, CHANNEL_WRITE_REGISTER, channel_val);
	HAL_Delay(5); // Random delays just in case
	uint8_t value = read_byte_data(I2C_DEV_ADDRESS, CHANNEL_READ_REGISTER);
	HAL_Delay(5); // Random delays just in case
	return value;
}

// TODO - implement this function
float get_value_channel(uint8_t channel) {

	/*
	We do START_CHANNEL_VAL + i*2 or START_CHANNEL_VAL + i*2 + 1
	because the data sheet tells us that the values to read from
	the channels are set up like this.
	*/
	// Get the msb value
	uint8_t msb = get_value(START_CHANNEL_VAL + channel*2);
	// Get the lsb value
	uint8_t lsb = get_value(START_CHANNEL_VAL + channel*2 + 1);

	// Shift the msb to align it where it belongs
	uint16_t msb_shifted = msb << 8;

	// Combine the shifted msb with the lsb
	uint16_t value = msb_shifted | lsb;
	return value;
}

void initialize_spectral() {
	// TODO - Fill out the value to write to channel write register.
	// Make all bit values to the default as shown in the data sheet except
	// for gain, which should be set to 16x.
	const uint8_t channel_write_val = TODO;
	write_byte_data(I2C_DEV_ADDRESS, CONTROL_SET_UP_REGISTER, channel_write_val);

	HAL_Delay(5); // Random delays just in case
	write_byte_data(I2C_DEV_ADDRESS, CONTROL_SET_UP_REGISTER, channel_write_val);

	// TODO - Fill out the value to write for integration time value.
	// Make the integration time as large as possible.
	const uint8_t integration_time_val = TODO;
	write_byte_data(I2C_DEV_ADDRESS, INTEGRATION_TIME_REGISTER, integration_time_val);
}

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // This will store data for spectral data
  uint16_t spectral_channel_data[6];

  // Initialize the spectral
  initialize_spectral();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // This will go through each of the six channels and get data
	  for (uint8_t i = 0; i < 6; ++i) {
		  spectral_channel_data[i] = get_value_channel(i);
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
