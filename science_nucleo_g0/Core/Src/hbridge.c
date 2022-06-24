/*
 * hbridge.c
 *
 *  Created on: Nov 29, 2020
 *      Orginal Author: cgiger
 *      hbridge-rewrite Author: jjtom
 */

#include <hbridge.h>
#include <math.h>

Motor *hbridge_motor;
int openloop_enabled = 0;

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
//	if (fabs(speed) > 1) {
//		speed = fabs(speed) / speed;
//	}

	HAL_GPIO_WritePin(motor->fwd_port, motor->fwd_pin, (speed > 0));
	HAL_GPIO_WritePin(motor->bwd_port, motor->bwd_pin, (speed < 0));
	if (speed == 0){
		// If speed = 0 then set both equal to each other for stop
		HAL_GPIO_WritePin(motor->fwd_port, motor->fwd_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->bwd_port, motor->bwd_pin, GPIO_PIN_RESET);
	}
	motor->timer->Instance->CCR1 = fabs(speed) * max;
//	HAL_Delay(100);

}

// Starting position 1
// 0-2
// -1 is error/ unknown state
int desPos;
int currPos;

void receive_hbridge_cmd(uint8_t *buffer) {
	// expects $CAROUSEL, <position>, <comma padding>
	char delim[] = ",";
	//TODO verify how legal this is to do in C
	char *copy = (char *)malloc(strlen(buffer) + 1);
	if (copy == NULL) {
	  return;
	}
	strncpy(copy, buffer,30);
	const char *identifier =  strtok(copy, delim);
	if (!strcmp(identifier, "$CAROUSEL")) {
		desPos = atoi(strtok(NULL, delim));
	}
}

void receive_carousel_openloop_cmd(uint8_t *buffer) {
	// expects $OPENCAROUSEL, <throttle>, <comma padding>
	char delim[] = ",";
	char *copy = (char *)malloc(strlen(buffer) + 1);
	if (copy == NULL) {
	  return;
	}
	strncpy(copy, buffer,30);
	const char *identifier =  strtok(copy, delim);
	if (!strcmp(identifier, "$OPENCAROUSEL")) {
		double throttle = (double)atoi(strtok(NULL, delim)) * 2.5;
		set_speed(hbridge_motor, throttle);

	}
}

void send_carousel_pos(UART_HandleTypeDef* huart){
	char string[20];
	sprintf((char *)string, "$CAROUSEL,%i,\n", currPos);
	 HAL_Delay(300);
	HAL_UART_Transmit_IT(huart, (uint8_t *)string, sizeof(string));
	HAL_Delay(100);
}
void handle_new_pos(){
	if (currPos == 2){
		if(desPos == 0){
			set_speed(hbridge_motor, -2.5);
		}
		else if (desPos == 1){
			set_speed(hbridge_motor, 2.5);
		}
		else if (desPos == 2){
			set_speed(hbridge_motor, 0);
		}
	}
	else if (currPos == 1){
		if(desPos != 1){
			set_speed(hbridge_motor, -2.5);
		}
		else{
			set_speed(hbridge_motor,0);
		}
	}
	else if (currPos == 0){
		if(desPos != 0){
			set_speed(hbridge_motor, 2.5);
		}
		else{
			set_speed(hbridge_motor,0);
		}
	}
	// TODO check if during travel you messed up and got unaligned
	// Alternatively, before travel go to 2, and on arrival go to one end and go to the desired position
	// *Assumes that you don't go past one of the end switches during travel
}
