/*
 * ammoniamotor.c
 *
 *  Created on: Nov 29, 2020
 *      Author: cgiger
 */

#include "ammonia_motor.h"

AmmoniaMotor *new_ammonia_motor(GPIO_TypeDef *fwd_port, uint16_t fwd_pin, GPIO_TypeDef *bwd_port,
												uint16_t bwd_pin, TIM_TypeDef *timer) {
	AmmoniaMotor *ammonia_motor = malloc(sizeof(AmmoniaMotor));
	ammonia_motor->fwd_port = fwd_port;
	ammonia_motor->bwd_port = bwd_port;
	ammonia_motor->fwd_pin = fwd_pin;
	ammonia_motor->bwd_pin = bwd_pin;
	ammonia_motor->timer = timer;
	ammonia_motor->timer->ARR = ARR;
	return ammonia_motor;
}

// pre scaler is 7
// pwm clk freq = 8/(7 + 1) = 1 MHz
// ARR = 15 us period = 67 KHz
// for LA max duty cycle is 20%
// CRR = 3 us max
//position ranges from -1 to 1
void set_pos(AmmoniaMotor *ammonia_motor, double pos) {
	ammonia_motor->timer->CCR1 = pos/100 * max;
	if (pos > 0) {
		HAL_GPIO_WritePin(ammonia_motor->fwd_port, ammonia_motor->fwd_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ammonia_motor->bwd_port, ammonia_motor->bwd_pin, GPIO_PIN_RESET);
	}
	else if (pos < 0) {
		HAL_GPIO_WritePin(ammonia_motor->bwd_port, ammonia_motor->fwd_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ammonia_motor->fwd_port, ammonia_motor->bwd_pin, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(ammonia_motor->bwd_port, ammonia_motor->fwd_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ammonia_motor->fwd_port, ammonia_motor->bwd_pin, GPIO_PIN_SET);
	}
}
