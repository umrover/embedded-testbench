/*
 * ammonia_motor.h
 *
 *  Created on: Nov 29, 2020
 *      Author: cgiger
 */

#ifndef SRC_AMMONIA_MOTOR_H_
#define SRC_AMMONIA_MOTOR_H_

#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <stdlib.h>

enum {
	ARR = 15,
	max = 3
};

typedef struct {
	GPIO_TypeDef *fwd_port;
	GPIO_TypeDef *bwd_port;
	TIM_TypeDef *timer;
	uint16_t fwd_pin;
	uint16_t bwd_pin;
} AmmoniaMotor;

AmmoniaMotor *new_ammonia_motor(GPIO_TypeDef *fwd_port, uint16_t fwd_pin, GPIO_TypeDef *bwd_port,
														uint16_t bwd_pin, TIM_TypeDef *timer);

void set_pos(AmmoniaMotor *, double);

#endif /* SRC_AMMONIA_MOTOR_H_ */
