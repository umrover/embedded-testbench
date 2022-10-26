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

void set_motor_speed(Motor *motor, float speed);

void update_limit_switches(Motor *motor);

void update_quad_encoder(Motor *motor);

void set_motor_angle(Motor *motor, float theta);
