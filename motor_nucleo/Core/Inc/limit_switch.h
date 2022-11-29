#pragma once

#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_it.h"

#include "pin.h"

typedef struct {
    Pin *pin;
    bool is_activated;
} LimitSwitch;

LimitSwitch *new_limit_switch(Pin *_pin);

void update_limit_switch(LimitSwitch *limit_switch);
