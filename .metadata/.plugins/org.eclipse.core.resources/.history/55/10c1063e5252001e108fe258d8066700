/*
 * servo.c
 *
 *  Created on: Sep 4, 2023
 *      Author: isabel
 */


#include "servo.h"

Servo *new_servo(TIM_HandleTypeDef* _timer, uint32_t _channel, uint32_t* _output)
{
	Servo *servo = (Servo*) malloc(sizeof(Servo));
	servo->timer = _timer;
	servo->channel = _channel;
	servo->output = _output;
    return servo;
}


void initialize_servo(Servo* servo, uint32_t initial_angle)
{
	HAL_TIM_PWM_Start(servo->timer, servo->channel);
	set_servo_angle(servo, initial_angle);
}


void set_servo_angle(Servo *servo, uint32_t angle)
{
    *(servo->output) = (angle/18)+10; // 20 ticks * 0.1ms/tick = 2ms -> 180 degrees
}
