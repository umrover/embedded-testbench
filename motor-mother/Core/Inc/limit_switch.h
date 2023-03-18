#pragma once

#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f1xx_hal.h"

#include "pin.h"

typedef struct {
    Pin *pin;
    bool enabled;
    bool is_activated;
    bool valid;
    bool active_high;
    int32_t associated_count;
} LimitSwitch;

LimitSwitch *new_limit_switch(bool _valid, Pin *_pin);

void update_limit_switch(LimitSwitch *limit_switch);
