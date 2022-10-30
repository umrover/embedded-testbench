/*
 * main_loop.c
 *
 *  Created on: Oct 31, 2021
 *      Author: aweso
 */

//Include for every Sensor
#include "main.h"
#include "main_loop.h"

// Extern Vars
extern char* mosfet_copy;
extern char* servo_copy;

#define JETSON_UART &huart2

int main_loop(){

#ifdef THERMISTOR_ENABLE
	// Read and send all thermistor data over Jetson UART
	send_thermistor_data(thermistors, JETSON_UART);
	if (auto_shutoff){
		for(int i = 0; i < THERMISTOR_DEVICES; ++i){
			if (curr_temps[i] >= max_temp){
				switch(i){
				  case THERMISTOR_OF_HEATER_0:
					  if (heater_state_0 == ENABLED) {
						  send_heater_info = 1;
						  enable_pin(DISABLED, HEATER_0_GPIO_Port, HEATER_0_Pin);
						  heater_state_0 = DISABLED;
					  }
					  break;
				  case THERMISTOR_OF_HEATER_1:
					  if (heater_state_1 == ENABLED) {
						  send_heater_info = 1;
						  enable_pin(DISABLED, HEATER_1_GPIO_Port, HEATER_1_Pin);
						  heater_state_1 = DISABLED;
					  }
					  break;
				  case THERMISTOR_OF_HEATER_2 :
					  if (heater_state_2 == ENABLED) {
						  send_heater_info = 1;
						  enable_pin(DISABLED, HEATER_2_GPIO_Port, HEATER_2_Pin);
					  	  heater_state_2 = DISABLED;
					  }
					  break;
				}
			}
		}
	}
#endif
#ifdef MOSFET_ENABLE
	if (send_heater_info) {
		send_heater_info = 0;
		send_heater_state(&huart2, 0, heater_state_0);
		send_heater_state(&huart2, 1, heater_state_1);
		send_heater_state(&huart2, 2, heater_state_2);
	}
	if (send_shutoff_info) {
		send_shutoff_info = 0;
		send_auto_shutoff_state(&huart2);
	}
#endif

#ifdef SPECTRAL_ENABLE

		initialize_spectral();
//	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
//		channel_select(mux, mux->channel_list[spectral_channels[i]]);

		//channel_select(mux, mux->channel_list[spectral_channels[i]]);
		// Before attempting read, check if that spectral device is ready
//		if(!check_ready()){
////			initialize_spectral();
//			continue;
//		}
		if (get_spectral_data(spectral, spectral_data)) {
			initialize_spectral();
		}

//	}
//	send_spectral_data(spectral_data, &huart2);
#endif

#ifdef TRIAD_ENABLE
	Channel channel;

	initialize_triad();
	// channel_select(mux_triad, mux_triad->channel_list[3]);  // UNCOMMENT if initialize_triad() is not being called in the line right before this one.

	// Check if triad is ready for reading
	// Triad is ready if everything is ready
	if(check_triad_ready()){
		for (uint8_t i = 0; i < 3; ++i) {
			virtual_write(DEV_SEL, triad[i]->dev_register);
			for (uint8_t j = 0; j < CHANNELS; ++j) {
				channel = *(triad[i]->channels[j]);
				channel.color_data = get_decimal(channel.lsb_register, channel.msb_register);
				if (i2c_bus_triad->ret != HAL_OK) {
					break;
				}
				// less complicated way
				triad_data[(i*CHANNELS) + j] = channel.color_data;

				HAL_Delay(10);
			}
		}

	}

	send_triad_data(triad_data, &huart2);
	HAL_Delay(10);
#endif

#ifdef HBRIDGE_MOTOR_ENABLE
    send_carousel_pos(&huart2);
#endif
	return 0;
}
