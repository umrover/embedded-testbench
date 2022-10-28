#ifndef BRIDGE_H_
#define BRIDGE_H_

#include "main.h"
#include "servo.h"
#include "adc_sensor.h"
#include "thermistor.h"
#include "mosfet.h"

// The communication bridge between the Jetson and the chip
typedef struct
{
	UART_HandleTypeDef *uart;
	ADCSensor *adc_sensor;
	Thermistor *science_thermistors[3];
	uint16_t science_temps[3];
	Servo *servos[3];
	MosfetDevice *mosfet_devices[8];
} Bridge;

// REQUIRES: uart is the uart channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Bridge object
Bridge *new_bridge(UART_HandleTypeDef *_uart);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Updates ADC and Themistor temperatures, and sets servos and mosfets
void bridge_iterate(Bridge *bridge);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends diagnostic current and thermistor data in format:
// $DIAG,,<TEMP_0>,<TEMP_1>,<TEMP_2>,<CURR_0>,<CURR_1>,<CURR_2>
void bridge_send_diagnostic(Bridge *bridge);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends science temperatures in format:
// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
void bridge_send_science_thermistors(Bridge *bridge);

/** PRIVATE FUNCTIONS MAY BE IN SOURCE FILE ONLY **/

#endif
