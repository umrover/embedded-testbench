#ifndef BRIDGE_H_
#define BRIDGE_H_

#include "main.h"
#include "servo.h"
#include "adc_sensor.h"
#include "thermistor.h"
#include "mosfet.h"
#include "spectral.h"

// The communication bridge between the Jetson and the chip
typedef struct
{
	UART_HandleTypeDef *uart;
} Bridge;

// REQUIRES: uart is the uart channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Bridge object
Bridge *new_bridge(UART_HandleTypeDef *_uart);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends diagnostic current and thermistor data in format:
// $DIAG,,<TEMP_0>,<TEMP_1>,<TEMP_2>,<CURR_0>,<CURR_1>,<CURR_2>
void bridge_send_diagnostic(Bridge *bridge, float temp_0, float temp_1, float temp_2,
							float current_0, float current_1, float current_2);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge, Spectral *spectral);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends science temperatures in format:
// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
void bridge_send_science_thermistors(Bridge *bridge, float temp_0, float temp_1, float temp_2);

/** PRIVATE FUNCTIONS MAY BE IN SOURCE FILE ONLY **/

#endif
