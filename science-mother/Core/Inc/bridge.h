#pragma once

#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

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
void bridge_send_diagnostic(Bridge *bridge, float temps[3], float currs[3]);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge, uint16_t ch_data[6]);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends science temperatures in format:
// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
void bridge_send_science_thermistors(Bridge *bridge, float temps[3]);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends heater auto shutoff in format:
// "$AUTOSHUTOFF,<VAL>"
void bridge_send_heater_auto_shutoff(Bridge *bridge, bool state);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends heater state in format:
// "$HEATER_DATA,<STATE_0>,<STATE_1>,<STATE_2>"
void bridge_send_heater_state(Bridge *bridge, bool states[3]);
