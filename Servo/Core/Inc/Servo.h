/*
 * Servo.h
 *
 *  Created on: Sep 21, 2023
 *      Author: wenboxu
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_
#pragma once
#include "stm32f3xx_hal.h"
#include <stdlib.h>

struct Servo
{
 TIM_HandleTypeDef *timer;
 uint32_t channel;
 uint32_t *output;
};


SERVO* new_servo(Servo ss,TIM_HandleTypeDef *timer, uint32_t channel, uint32_t *output){
ss->timer=timer;
ss->channel=channel;
ss->output=output;
return ss;
}

void intialize_servo(TIM_HandleTypeDef *timer,uint32_t channel, int startingangle){
 HAL_TIM_PWM_Start(*timer,channel);
 set_servo_angle(startingangle);
}

void set_servo_angle(int angle){
int neededticks= 450+(225*angle/90);


}
#endif /* INC_SERVO_H_ */
