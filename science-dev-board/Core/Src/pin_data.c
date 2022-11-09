#include "pin_data.h"

/** PUBLIC FUNCTIONS **/

// REQUIRES: _port and _pin corresponds to
// the port and pin.
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created PinData object
PinData *new_pin_data(GPIO_TypeDef *_port, uint16_t _pin) {
    PinData *pin_data = (PinData*) malloc(sizeof(PinData));
	pin_data->port = _port;
    pin_data->pin = _pin;
	return pin_data;
}

// REQUIRES: pin_data is PinData and value is 0 or 1
// MODIFIES: nothing
// EFFECTS: Sets pin to value
void set_pin_value(PinData *pin_data, uint8_t value) {
	HAL_GPIO_WritePin(pin_data->port, pin_data->pin, value == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/** PRIVATE FUNCTIONS MAY BE IN SOURCE FILE ONLY **/
