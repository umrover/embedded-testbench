#include "pin.h"

Pin* new_pin(GPIO_TypeDef *_port, uint16_t _pin)
{
	Pin* pin = (Pin*) malloc(sizeof(Pin));
	pin->port = _port;
	pin->pin = _pin;

	return pin;
}
