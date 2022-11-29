#pragma once

#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_it.h"

#include "pin.h"
#include "hbridge.h"
#include "control.h"
#include "quad_encoder.h"

typedef struct {
    HBridge *hbridge;
    Pin *forward_limit_switch_pin;
    Pin *backward_limit_switch_pin;
    QuadEncoder *encoder;
    Control *control;

    bool at_fwd_lim;
    bool at_rev_lim;
    float desired_speed;
    int32_t desired_counts;
    uint8_t mode;
    float speed_limit;
    bool limit_enabled;
    bool calibrated;
} Motor;

Motor *new_motor(HBridge *_hbridge, Pin *_fwd_lim, Pin *_bwd_lim, QuadEncoder *_encoder, Control *_control);

void initialize_motor(Motor *motor, float speed);

void set_motor_speed(Motor *motor, float speed);

void update_motor_speed(Motor *motor);

void update_motor_limits(Motor *motor);

void move_motor_to_target(Motor *motor, int32_t counts, float dt);

