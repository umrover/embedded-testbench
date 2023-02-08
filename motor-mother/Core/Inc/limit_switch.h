#pragma once

#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f1xx_hal.h"

#include "pin.h"

typedef struct {
    Pin *pin;
    float config_counts;
    bool is_activated;
} LimitSwitch;

LimitSwitch *new_limit_switch(Pin *_pin);

void update_limit_switch(LimitSwitch *limit_switch);
