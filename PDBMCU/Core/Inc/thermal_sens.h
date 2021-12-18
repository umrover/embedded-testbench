/*
 * thermal_sens.h
 *
 *  Created on: Apr 15, 2021
 */

#ifndef INC_THERMAL_SENS_H_
#define INC_THERMAL_SENS_H_

#include "stm32g0xx.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "smbus.h"

// Thermal Sensor: MCP9808T-E/MS

// Thermal Sensor Struct

// This struct stores data regarding the thermal sensor itself
typedef struct {

	// store the address
	int address;

	// store the i2cBus data
	SMBus* i2cBus;

} ThermalSensor;

//
//
// All Public Functions

// EFFECTS: Create a new ThermalSensor struct and returns pointer to struct.
ThermalSensor* new_thermal_sensor(SMBus* _i2cBus, uint8_t A0, uint8_t A1, uint8_t A2);

// EFFECTS: Get temperature data from a thermal sensor in Celsius
float get_thermal_data(const ThermalSensor* _thermal_sensor);

// EFFECTS: Delete the ThermalSensor object from memory
void delete_thermal_sensor(ThermalSensor* _thermal_sensor);

//
//
// All Private Functions


#endif /* INC_THERMAL_SENS_H_ */
