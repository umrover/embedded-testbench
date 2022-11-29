#pragma once

#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_it.h"

#include "limit_switch.h"
#include "pin.h"
#include "hbridge.h"
#include "control.h"
#include "quad_encoder.h"

typedef struct {
    HBridge *hbridge;
    LimitSwitch *forward_limit_switch;
    LimitSwitch *backward_limit_switch;
    QuadEncoder *encoder;
    Control *control;

    float desired_speed;
    int32_t desired_counts;
    uint8_t mode;
    float speed_limit;
    bool limit_enabled;
    bool calibrated;
} Motor;

Motor *new_motor(HBridge *_hbridge, LimitSwitch *_fwd_lim, LimitSwitch *_bwd_lim, QuadEncoder *_encoder, Control *_control);

void init_motor(Motor *motor, float speed);

void set_motor_speed(Motor *motor, float speed);

void update_motor_speed(Motor *motor);

void update_motor_limits(Motor *motor);

void move_motor_to_target(Motor *motor, int32_t counts, float dt);

