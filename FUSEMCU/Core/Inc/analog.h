#ifndef INC_ANALOG_H_
#define INC_ANALOG_H_

#include "stm32g0xx.h"
#include <stdlib.h>

#define VOLTAGE_DEVICES 12
#define CURRENT_DEVICES 12

// There are 10 pins and ports used to get data
extern int16_t pin_array[10];
extern GPIO_TypeDef* port_array[10];

typedef enum {
	Voltage_Select_0,		// 0
	Voltage_Select_1,		// 1
	Voltage_Select_2,		// 2
	Voltage_Select_3,		// 3
	Voltage_Enable			// 4
} VOLTAGE_NAME;

typedef enum {
	Current_Enable,			// 0
	Current_Select_3,		// 1
	Current_Select_2,		// 2
	Current_Select_1,		// 3
	Current_Select_0		// 4
} CURRENT_NAME;

// Current Sensor: ACS722LLCTR-10AU-T

// Analog Struct

typedef struct {

	// Stores the four adc_pins
	ADC_HandleTypeDef* in;
	uint8_t select_pins[4];

} Analog;

// All Public Functions

// NOTE: the way we get stuff from ADC is different here than in the PDB code
// EFFECTS: Create a new Analog object and returns pointer to object.
// TODO
Analog* new_analog(ADC_HandleTypeDef* in, uint8_t S0, uint8_t S1, uint8_t S2, uint8_t S3);

// EFFECTS: Get voltage data from a specific voltage sensor in Volts
float get_voltage_data(const Analog* _Analog);

// EFFECTS: Get current data from a specific current sensor in Amps
float get_current_data(const Analog* _Analog);

// EFFECTS: Delete the Analog object from memory
void delete_analog(Analog* _Analog);

//
//
// All Private Functions

// EFFECTS: Read from ADC object
// Returns a number between 0 and 4095
uint32_t read_from_ADC(ADC_HandleTypeDef* adc_object);

#endif /* INC_ANALOG_H_ */
