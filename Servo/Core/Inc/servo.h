#pragma once

#include "stm32f3xx_hal.h"

#include <stdlib.h>

typedef struct{

	TIM_HandleTypeDef* timer;
	uint32_t channel;
	uint32_t *output;

} Servo;

Servo* new_servo(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t *output);

void initialize_servo(Servo* servo, double angle);

void set_servo_angle(Servo* servo, double angle);
