#include "limit_switch.h"


LimitSwitch *new_limit_switch(Pin *_pin) {
	LimitSwitch *limit_switch = (LimitSwitch *) malloc(sizeof(LimitSwitch));
	limit_switch->pin = _pin;
	limit_switch->is_activated = false;
	return limit_switch;
}

void update_limit_switch(LimitSwitch *limit_switch) {
	limit_switch->is_activated = read_pin_value(limit_switch->pin);
}
