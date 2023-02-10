#include "limit_switch.h"


LimitSwitch *new_limit_switch(Pin *_pin) {
	LimitSwitch *limit_switch = (LimitSwitch *) malloc(sizeof(LimitSwitch));
	limit_switch->pin = _pin;
	limit_switch->config_counts = 0;
	limit_switch->is_activated = false;
	limit_switch->has_been_activated = false;
	return limit_switch;
}

void update_limit_switch(LimitSwitch *limit_switch) {
	if (read_pin_value(limit_switch->pin)) {
		limit_switch->is_activated = false;
	} else {
		limit_switch->is_activated = true;
	}
}
