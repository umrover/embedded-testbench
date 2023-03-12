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
#include "abs_enc_reading.h"

typedef struct {
    HBridge *hbridge;
    LimitSwitch *forward_limit_switch;
    LimitSwitch *backward_limit_switch;
    QuadEncoder *encoder;
    AbsEncoder *abs_encoder;
    ClosedLoopControl *control;

    bool valid;
    bool using_open_loop_control;
    float output_pwm; // USE FOR PWM! Should be between -max_pwm and max_pwm
    float max_pwm;  // A configuration value! Should be between 0 and 1
    float desired_speed; // Do not use raw value for PWM! Should be between -1 and 1
    int32_t desired_counts;
    bool limit_enabled;
    bool calibrated;
} Motor;

Motor *new_motor(bool _valid, HBridge *_hbridge, LimitSwitch *_fwd_lim, LimitSwitch *_bwd_lim, QuadEncoder *_encoder, AbsEncoder *_abs_encoder, ClosedLoopControl *_control);

void init_motor(Motor *motor, float speed);

void update_motor_target(Motor *motor);

void set_motor_speed(Motor *motor, float speed);

void update_motor_speed(Motor *motor);

void update_motor_limits(Motor *motor);

void move_motor_to_target(Motor *motor);

void refresh_motor_absolute_encoder_value(Motor *motor);

