#pragma once

#include <closed_loop_control.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f1xx_hal.h"

#include "limit_switch.h"
#include "pin.h"
#include "hbridge.h"
#include "quad_encoder.h"

typedef struct {
    HBridge *hbridge;
    LimitSwitch *forward_limit_switch;
    LimitSwitch *backward_limit_switch;
    QuadEncoder *encoder;
    ClosedLoopControl *control;

    bool using_open_loop_control;
    float output_pwm; // should be between -1 and 1
    float max_pwm;  // should be between -1 and 1
    float desired_speed; // should be between -1 and 1
    int32_t desired_counts;
    bool limit_enabled;
    bool calibrated;
} Motor;

Motor *new_motor(HBridge *_hbridge, LimitSwitch *_fwd_lim, LimitSwitch *_bwd_lim, QuadEncoder *_encoder, ClosedLoopControl *_control);

void init_motor(Motor *motor, float speed);

void update_motor_target(Motor *motor);

void set_motor_speed(Motor *motor, float speed);

void update_motor_speed(Motor *motor);

void update_motor_limits(Motor *motor);

void move_motor_to_target(Motor *motor);

