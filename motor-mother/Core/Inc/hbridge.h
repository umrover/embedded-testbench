#pragma once

#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_it.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} Pin;

typedef struct {
	TIM_HandleTypeDef *timer;
	uint32_t channel;
	uint32_t *out_register;
	uint32_t ARR;
    Pin *fwd;
    Pin *bwd;
} HBridge;


Pin* new_pin(GPIO_TypeDef *_port, uint16_t _pin);

// Returns pointer to a new hbridge object
HBridge* new_hbridge(TIM_HandleTypeDef *_timer, uint32_t _channel, uint32_t *_out_register, uint32_t _ARR, Pin* _fwd, Pin* _bwd);


// Initialize timer settings
void initialize_hbridge(HBridge* hbridge, double duty_cycle, uint8_t direction);


// Requires a signal of between 0 and 1 for duty cycle
// Calculates high/low pulse durations and sends to hbridge
void set_pwm(HBridge* hbridge, double duty_cycle);


// REQUIRES: direction to be -66 to 66
// MODIFIES: nothing
// EFFECTS: sets fwd pin and bwd pin as (0,1) or (1,0)
void set_dir(HBridge* hbridge, int8_t direction);
