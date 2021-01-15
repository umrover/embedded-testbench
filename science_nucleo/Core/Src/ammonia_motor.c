/*
 * ammoniamotor.c
 *
 *  Created on: Nov 29, 2020
 *      Author: cgiger
 */

#include "ammonia_motor.h"
#include <math.h>

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

// pre scaler is 799
// pwm clk freq = 8/(799 + 1) = 10 KHz
// ARR =  200 for a 20 ms period = 20 KHz
// for LA max duty cycle is 20%
// CRR = 20 us max
// speed ranges from -1 to 1
void set_pos(AmmoniaMotor *ammonia_motor, double speed) {
	if (fabs(speed) > 1) {
		speed = fabs(speed) / speed;
	}
	ammonia_motor->timer->CCR1 = fabs(speed) * max;

	HAL_GPIO_WritePin(ammonia_motor->fwd_port, ammonia_motor->fwd_pin, (speed > 0) | (speed == 0));
	HAL_GPIO_WritePin(ammonia_motor->bwd_port, ammonia_motor->bwd_pin, (speed < 0) | (speed == 0));

}
