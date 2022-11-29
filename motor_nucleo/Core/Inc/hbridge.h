#pragma once

#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_it.h"
#include "pin.h"

typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    uint32_t *out_register;
    uint32_t ARR;
    Pin *forward_pin;
    Pin *backward_pin;
    uint32_t target_duty_cycle;
} HBridge;


// Returns pointer to a new hbridge object
HBridge *new_hbridge(
		TIM_HandleTypeDef *_timer,
		uint32_t _channel,
		uint32_t *_out_register,
		uint32_t _ARR,
		Pin *fwd,
		Pin *bwd);

// Initialize timer settings
void init_hbridge(HBridge *hbridge, float duty_cycle, uint8_t direction);

// Requires a signal of between 0 and 1 for duty cycle
// Calculates high/low pulse durations and sends to hbridge
void set_pwm(HBridge *hbridge, float duty_cycle);

// REQUIRES: direction to be -66 to 66
// MODIFIES: nothing
// EFFECTS: sets fwd pin and bwd pin as (0,1) or (1,0)
void set_dir(HBridge *hbridge, float speed);

