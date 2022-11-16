#pragma once

#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_it.h"

#include "hbridge.h"
#include "control.h"

typedef struct
{
	Pin *pin;
} LimitSwitch;

typedef struct
{
	TIM_HandleTypeDef *htim;
	TIM_TypeDef *tim;
	int32_t raw;
	int16_t PPR;
} QuadEncoder;

typedef struct
{
	HBridge *hbridge;
	LimitSwitch *fwd_lim;
	LimitSwitch *rev_lim;
	QuadEncoder *encoder;
	Gains *gains;
	bool at_fwd_lim;
	bool at_rev_lim;
	int32_t angle;
	int32_t raw_angle;
	float desired_speed;

} Motor;

LimitSwitch* new_limit_switch(Pin* _pin);

QuadEncoder* new_quad_encoder(TIM_HandleTypeDef *_htim, TIM_TypeDef *_tim, int16_t _PPR);

Motor* new_motor(HBridge *_hbridge, LimitSwitch* _fwd_lim, LimitSwitch* _rev_lim, QuadEncoder* _encoder, Gains* _gains);


void initialize_motor(Motor* motor, float speed, float theta);


void set_motor_speed(Motor *motor, float speed);


void update_motor_speed(Motor* motor);


void update_limit_switches(Motor *motor);


void update_quad_encoder(Motor *motor);


void set_motor_angle(Motor *motor, float angle, float dt);


void motor_periodic(TIM_HandleTypeDef *htim, TIM_HandleTypeDef *central_tim, Motor* motor[], int num_motors);
