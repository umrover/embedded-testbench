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
	max = 40
};

typedef struct {
	GPIO_TypeDef *fwd_port;
	GPIO_TypeDef *bwd_port;
	TIM_HandleTypeDef *timer;
	uint16_t fwd_pin;
	uint16_t bwd_pin;
} Motor;

Motor *new_motor(GPIO_TypeDef *fwd_port, uint16_t fwd_pin, GPIO_TypeDef *bwd_port,
														uint16_t bwd_pin, TIM_HandleTypeDef *timer);
void start(Motor *motor, int channel);

void set_speed(Motor *motor, double speed);

#endif /* SRC_AMMONIA_MOTOR_H_ */
