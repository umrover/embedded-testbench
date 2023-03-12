#include "limit_switch.h"


LimitSwitch *new_limit_switch(bool _valid, Pin *_pin) {
	LimitSwitch *limit_switch = (LimitSwitch *) malloc(sizeof(LimitSwitch));
	limit_switch->valid = _valid;
	limit_switch->pin = _pin;
	limit_switch->is_activated = false;
	return limit_switch;
}

void update_limit_switch(LimitSwitch *limit_switch) {
	if (read_pin_value(limit_switch->pin)) {
		limit_switch->is_activated = 0;
	} else {
		limit_switch->is_activated = 1;
	}
}
