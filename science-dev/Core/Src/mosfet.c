/*
 * mosfet.c
 *
 *  Created on: Sep 25, 2022
 *      Author: ASD
 */

#include "mosfet.h"

MosfetDevice *new_mosfet_device(GPIO_TypeDef *_port, uint8_t _pin) {
	MosfetDevice new_m_dev = {_port, _pin};
	MosfetDevice *new_m_dev_ptr = &new_m_dev;
	return new_m_dev_ptr;
}

void turn_mosfet_device_on(MosfetDevice *mosfet_device) {
	GPIO_TypeDef *temp_port = mosfet_device->port;
	uint8_t temp_pin = mosfet_device->pin;
	HAL_GPIO_WritePin(temp_port, temp_pin, GPIO_PIN_SET);
}

void turn_mosfet_device_off(MosfetDevice *mosfet_device) {
	GPIO_TypeDef *temp_port = mosfet_device->port;
	uint8_t temp_pin = mosfet_device->pin;
	HAL_GPIO_WritePin(temp_port, temp_pin, GPIO_PIN_RESET);
}

void set_mosfet_device_state(MosfetDevice *mosfet_device, uint8_t state) {
	if(state == 1) {
		turn_mosfet_device_on(mosfet_device);
	}
	else {
		turn_mosfet_device_off(mosfet_device);
	}
}
