/*
 * ammonia_motor.h
 *
 *  Created on: Nov 29, 2020
 *      Author: cgiger
 */

#ifndef HBRIDGE_MOTOR_ENABLE
#define HBRIDGE_MOTOR_ENABLE

#include "stm32g0xx_hal.h"
#include "main.h"
#include <stdio.h>
#include "string.h"
#include <stdlib.h>

enum {
	max = 160
};

typedef struct {
	GPIO_TypeDef *fwd_port;
	GPIO_TypeDef *bwd_port;
	TIM_HandleTypeDef *timer;
	uint16_t fwd_pin;
	uint16_t bwd_pin;
} Motor;


// Motor Specific Externs
extern Motor *hbridge_motor;

// Motor Specific definitions
Motor *new_motor(GPIO_TypeDef *fwd_port, uint16_t fwd_pin, GPIO_TypeDef *bwd_port,
														uint16_t bwd_pin, TIM_HandleTypeDef *timer);
void start(Motor *motor, int channel);

void set_speed(Motor *motor, double speed);

void receive_hbridge_cmd(uint8_t *buffer);

// Limit Switch Position
extern int currPos;
extern int desPos;
extern int openloop_enabled;

void receive_carousel_openloop_cmd(uint8_t *buffer);

// Send carousel position back to jetson
void send_carousel_pos(UART_HandleTypeDef* huart);

// Move carousel after receiving a message
void handle_new_pos();
#endif /* HBRIDGE_MOTOR_ENABLE */
