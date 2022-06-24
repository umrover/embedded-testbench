/*
 * main_preloop.c
 *
 *  Created on: Oct 31, 2021
 *      Author: aweso
 */

//Include for every Sensor
#include "main.h"
#include "main_loop.h"

#define JETSON_UART &huart2

int main_preloop(){
	HAL_UART_Receive_IT(JETSON_UART,Rx_data,30);
#ifdef THERMISTOR_ENABLE
	thermistors = new_thermistors(ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5);
#endif

#ifdef MOSFET_ENABLE
	HAL_TIM_Base_Start_IT(&htim6); // needed for LED timer
	mosfet_copy = (char *)malloc(30);
	auto_shut_copy = (char *)malloc(30);
#endif

#ifdef MUX_ENABLE
  	i2c_bus = new_smbus(&hi2c1, JETSON_UART);
  	i2c_bus_triad = new_smbus(&hi2c1, JETSON_UART);
  	disable_DMA(i2c_bus);
  	disable_DMA(i2c_bus_triad);
  	mux = new_mux(i2c_bus);
  	mux_triad = new_mux(i2c_bus_triad);

#endif

#ifdef SERVO_ENABLE
  	write_angle(INITIAL_SERVO_0_ANGLE, 0);
	write_angle(INITIAL_SERVO_1_ANGLE, 1);
	write_angle(INITIAL_SERVO_2_ANGLE, 2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	servo_copy = (char *)malloc(30);
#endif

#ifdef SPECTRAL_ENABLE
	HAL_Delay(500);

  	spectral_exists = 1;

	spectral = new_spectral(i2c_bus);

	// adds all the spectral channels
	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
		add_channel(mux, spectral_channels[i]);
	}

	initialize_spectral();

#endif

#ifdef HBRIDGE_MOTOR_ENABLE
	hbridge_motor = new_motor(HBRIDGE_FWD_GPIO_Port, HBRIDGE_FWD_Pin, HBRIDGE_BWD_GPIO_Port, HBRIDGE_BWD_Pin, &htim3);
	start(hbridge_motor, TIM_CHANNEL_1);
	set_speed(hbridge_motor, 0);
#endif

#ifdef TRIAD_ENABLE

	triad_exists = 1;

	add_channel(mux_triad, 3);

	initialize_triad();
#endif

	return 0;
}
