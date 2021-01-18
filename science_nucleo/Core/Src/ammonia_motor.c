/*
 * ammoniamotor.c
 *
 *  Created on: Nov 29, 2020
 *      Author: cgiger
 */

#include "ammonia_motor.h"
#include <math.h>

Motor *new_motor(GPIO_TypeDef *fwd_port, uint16_t fwd_pin, GPIO_TypeDef *bwd_port,
												uint16_t bwd_pin, TIM_HandleTypeDef *timer) {
	Motor *motor = malloc(sizeof(Motor));
	motor->fwd_port = fwd_port;
	motor->bwd_port = bwd_port;
	motor->fwd_pin = fwd_pin;
	motor->bwd_pin = bwd_pin;
	motor->timer = timer;
	return motor;
}


void start(Motor *motor, int channel) {
	HAL_TIM_PWM_Start(motor->timer, channel);
}
// pre scaler is 799
// pwm clk freq = 8/(799 + 1) = 10 KHz
// ARR =  200 for a 20 ms period = 20 KHz
// for LA max duty cycle is 20%
// CRR = 20 us max
// speed ranges from -1 to 1
void set_speed(Motor *motor, double speed) {
	// handles speed being > 1
	if (fabs(speed) > 1) {
		speed = fabs(speed) / speed;
	}
	HAL_GPIO_WritePin(motor->fwd_port, motor->fwd_pin, (speed > 0) | (speed == 0));
	HAL_GPIO_WritePin(motor->bwd_port, motor->bwd_pin, (speed < 0) | (speed == 0));

	motor->timer->Instance->CCR1 = fabs(speed) * max;
	HAL_Delay(1);

}
