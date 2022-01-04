/*
 * temperature_sens.h
 *
 *  Created on: Apr 15, 2021
 */

#ifndef INC_TEMPERATURE_SENS_H_
#define INC_TEMPERATURE_SENS_H_

#include "stm32g0xx.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "smbus.h"

// Temperature Sensor: MCP9808T-E/MS

// Temperature Sensor Struct

// This struct stores data regarding the temperature sensor itself
typedef struct {

	// store the address
	int address;

	// store the i2cBus data
	SMBus* i2cBus;

} TemperatureSensor;

//
//
// All Public Functions

// EFFECTS: Create a new TemperatureSensor struct and returns pointer to struct.
TemperatureSensor* new_temperature_sensor(SMBus* _i2cBus, uint8_t A0, uint8_t A1, uint8_t A2);

// EFFECTS: Get temperature data from a temperature sensor in Celsius
float get_temperature_data(const TemperatureSensor* _temperature_sensor);

// EFFECTS: Delete the TemperatureSensor object from memory
void delete_temperature_sensor(TemperatureSensor* _temperature_sensor);

//
//
// All Private Functions


#endif /* INC_TEMPERATURE_SENS_H_ */
