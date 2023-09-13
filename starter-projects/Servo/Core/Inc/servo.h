/*
 * servo.h
 *
 *  Created on: Sep 4, 2023
 *      Author: isabel
 */

#ifndef SERVO_H_
#define SERVO_H_

#pragma once

#include "stm32f3xx_hal.h"
#include <stdlib.h>

typedef struct {
	TIM_HandleTypeDef *timer;
	uint32_t channel;
	uint32_t *output;
} Servo;

Servo *new_servo(TIM_HandleTypeDef* _timer, uint32_t _channel, uint32_t* _output);

void initialize_servo(Servo* servo, uint32_t intial_angle);

void set_servo_angle(Servo* servo, uint32_t angle);

#endif /* SERVO_H_ */
