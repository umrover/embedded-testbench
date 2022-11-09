#pragma once

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdlib.h>

// Used to store the data of a pin
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} PinData;

/** PUBLIC FUNCTIONS **/

// REQUIRES: _port and _pin corresponds to
// the port and pin.
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created PinData object
PinData *new_pin_data(GPIO_TypeDef *_port, uint16_t _pin);

// REQUIRES: pin_data is PinData and value is 0 or 1
// MODIFIES: nothing
// EFFECTS: Sets pin to value
void set_pin_value(PinData *pin_data, uint8_t value);

/** PRIVATE FUNCTIONS MAY BE IN SOURCE FILE ONLY **/
