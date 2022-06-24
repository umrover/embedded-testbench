/*
 * main_loop.h
 *
 *  Created on: Oct 31, 2021
 *      Author: aweso
 */

#ifndef INC_MAIN_LOOP_H_
#define INC_MAIN_LOOP_H_

#include "thermistor.h"
#include "main.h"
#include "spectral.h"
#include "hbridge.h"
#include "mosfet.h"
#include "servo.h"
#include "triad.h"
#include "main_preloop.h"

// Defines
#define THERMISTOR_ENABLE
#define MOSFET_ENABLE
#define SERVO_ENABLE
#define SPECTRAL_ENABLE
// #define HBRIDGE_MOTOR_ENABLE
// #define TRIAD_ENABLE
#define MUX_ENABLE

int main_loop();

#endif /* INC_MAIN_LOOP_H_ */
