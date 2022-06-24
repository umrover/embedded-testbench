/*
 * servo.h
 *
 *  Created on: Dec 2, 2021
 *      Author: Sashreek
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"
#include "stdlib.h"
#include "string.h"

#define INITIAL_SERVO_0_ANGLE 80
#define INITIAL_SERVO_1_ANGLE 100
#define INITIAL_SERVO_2_ANGLE 100

// THIS ORDER IS NOT A MISTAKE. CHANGE IT AROUND IF THE SERVOS ARE CONTROLLING THE WRONG SERVO.
typedef enum  // gui 2 is actually 1, gui 1 is 0, gui 0 is 2
{
	DEVICE_OF_SERVO_1,
	DEVICE_OF_SERVO_2,
	DEVICE_OF_SERVO_0,
	SERVO_DEVICES
} SERVO_DEVICE_NAME;

extern char*servo_copy;
void write_angle(int angle, int servo_num);
void receive_servo_cmd(uint8_t *buffer, int *angle_0, int *angle_1, int *angle_2, char*servo_copy);

#endif /* INC_SERVO_H_ */
