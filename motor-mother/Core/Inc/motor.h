#pragma once

#include <stdlib.h>
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_it.h"

#include "hbridge.h"

typedef struct
{
	Pin *pin;
} LimitSwitch;

typedef struct
{
	TIM_HandleTypeDef *timer;
	uint32_t *channel_A;
	uint32_t *channel_B;
} QuadEncoder;

typedef struct
{
	HBridge *hbridge;
	LimitSwitch *fwd_lim;
	LimitSwitch *rev_lim;
	QuadEncoder *encoder;
	bool at_fwd_lim;
	bool at_rev_lim;
	float angle;
} Motor;

LimitSwitch* new_limit_switch(Pin* _pin);

QuadEncoder* new_quad_encoder(TIM_HandleTypeDef* _timer, uint32_t* _channel_A, uint32_t* _channel_B);

Motor* new_motor(HBridge *_hbridge, LimitSwitch* _fwd_lim, LimitSwitch* _rev_lim, QuadEncoder* _encoder, bool _at_fwd_lim, bool _at_rev_lim, float _angle);


void initialize_motor(Motor* motor, float speed, float theta);


void set_motor_speed(Motor *motor, float speed);


void update_limit_switches(Motor *motor);


void update_quad_encoder(Motor *motor);


void set_motor_angle(Motor *motor, float theta);
