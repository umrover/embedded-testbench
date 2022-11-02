#pragma once

#include <stdlib.h>
#include "stdbool.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_it.h"

#include "hbridge.h"

typedef struct
{
	Pin *pin;
} LimitSwitch;

typedef struct
{
	TIM_HandleTypeDef *htim;
	TIM_TypeDef *tim;
	int16_t raw;
	int16_t prev_raw;
} QuadEncoder;

typedef struct
{
	HBridge *hbridge;
	LimitSwitch *fwd_lim;
	LimitSwitch *rev_lim;
	QuadEncoder *encoder;
	bool at_fwd_lim;
	bool at_rev_lim;
	int16_t position;
} Motor;

LimitSwitch* new_limit_switch(Pin* _pin);

QuadEncoder* new_quad_encoder(TIM_HandleTypeDef *_htim, TIM_TypeDef *_tim);

Motor* new_motor(HBridge *_hbridge, LimitSwitch* _fwd_lim, LimitSwitch* _rev_lim, QuadEncoder* _encoder, int16_t _position);


void initialize_motor(Motor* motor, float speed, float theta);


void set_motor_speed(Motor *motor, float speed);


void update_limit_switches(Motor *motor);


void update_quad_encoder(Motor *motor);


void set_motor_angle(Motor *motor, float theta);


void motor_periodic(TIM_HandleTypeDef *htim, TIM_HandleTypeDef *central_tim, Motor* motor[], int num_motors);
