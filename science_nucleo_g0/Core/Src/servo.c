/*
 * servo.c
 *
 *  Created on: Dec 2, 2021
 *      Author: Sashreek
 */

#include "servo.h"

void write_angle(int angle, int servo_num){

	if (servo_num == 0){
		TIM1->CCR1 = (angle/18)+15;
	}
	else if (servo_num == 1){
		TIM1->CCR2 = (angle/18)+15;
	}
	else if (servo_num == 2){
		TIM1->CCR3 = (angle/18)+15;
	}
}

char*servo_copy;

void receive_servo_cmd(uint8_t *buffer, int *angle_0, int *angle_1, int *angle_2, char*servo_copy){

  //Change to string
  char delim[] = ",";

  strncpy(servo_copy, buffer,30);

  if (servo_copy == NULL) {
  	return;
  }

  //Expected $SERVO,<angle_0>,<angle_1>,<angle_2>
  char *identifier = strtok(servo_copy,delim);
  if (!strcmp(identifier,"$SERVO")){
	  *angle_0 = atoi(strtok(NULL,delim));
	  *angle_1 = atoi(strtok(NULL,delim));
	  *angle_2 = atoi(strtok(NULL,delim));
  }
}
