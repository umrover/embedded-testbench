#include "mosfet.h"

// REQUIRES: _port is the port and _pin is the pin
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Mosfet Device object
MosfetDevice *new_mosfet_device(GPIO_TypeDef *_port, uint8_t _pin)
{
	MosfetDevice *new_m_dev = malloc(sizeof(MosfetDevice));
	new_m_dev->port = _port;
	new_m_dev->pin = _pin;
	return new_m_dev;
}

// REQUIRES: mosfet_device is a Mosfet Device object
// MODIFIES: nothing
// EFFECTS: Turns Mosfet Device on by setting pin high
void turn_mosfet_device_on(MosfetDevice *mosfet_device)
{
	GPIO_TypeDef *temp_port = mosfet_device->port;
	uint8_t temp_pin = mosfet_device->pin;
	HAL_GPIO_WritePin(temp_port, temp_pin, GPIO_PIN_SET);
}

// REQUIRES: mosfet_device is a Mosfet Device object
// MODIFIES: nothing
// EFFECTS: Turns Mosfet Device off by setting pin low
void turn_mosfet_device_off(MosfetDevice *mosfet_device)
{
	GPIO_TypeDef *temp_port = mosfet_device->port;
	uint8_t temp_pin = mosfet_device->pin;
	HAL_GPIO_WritePin(temp_port, temp_pin, GPIO_PIN_RESET);
}

// REQUIRES: mosfet_device is a Mosfet Device object and state is either 0 or 1
// MODIFIES: nothing
// EFFECTS: Sets Mosfet Device to desired state
void set_mosfet_device_state(MosfetDevice *mosfet_device, bool state)
{
	if (state)
	{
		turn_mosfet_device_on(mosfet_device);
	}
	else
	{
		turn_mosfet_device_off(mosfet_device);
	}
}
