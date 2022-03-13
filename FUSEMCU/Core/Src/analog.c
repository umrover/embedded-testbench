#ifndef INC_ANALOG_H_

#include "analog.h"
#include "main.h"

// Current Sensor: ACS722LLCTR-10AU-T

// EFFECTS: Create a new Analog object and returns pointer to object.
// Takes in 'pins' array that specify which of the 4 pins are selected,
// depending on device number
// TODO
Analog* new_analog(ADC_HandleTypeDef* in, uint8_t S0, uint8_t S1, uint8_t S2, uint8_t S3) {
	// Create a new struct
	Analog* Analog_object = (Analog*) malloc(sizeof(Analog));

	Analog_object->in = in;
	Analog_object->select_pins[0] = S0;
	Analog_object->select_pins[1] = S1;
	Analog_object->select_pins[2] = S2;
	Analog_object->select_pins[3] = S3;

	return Analog_object;

}

// EFFECTS: Get voltage data from a specific voltage sensor in Volts
float get_voltage_data(const Analog* _Analog){
	// Read data from ADC, raw_ADC_data is a number in [0, 4095]
	uint32_t raw_ADC_data = read_from_ADC(_Analog->in);
	// Convert from [0, 4095] to [0, 1] to [0, 3.3]
	float steppedDownVoltage = (raw_ADC_data / 4095.0) * 3.3;
	// Convert steppedDownVoltage to actual output voltage (multiplier of 14)
	return steppedDownVoltage * 14.0;
}

// EFFECTS: Get current data from a specific current sensor in Amps
float get_current_data(const Analog* _Analog){
	// Read data from ADC, raw_ADC_data is a number in [0, 4095]
	uint32_t raw_ADC_data = read_from_ADC(_Analog->in);
	return raw_ADC_data;
}

// EFFECTS: Delete the Analog object from memory
void delete_analog(Analog* _Analog){
	free(_Analog);
}

//
//
// All Private Functions

// EFFECTS: Read from ADC object
// Returns a number between 0 and 4095
uint32_t read_from_ADC(ADC_HandleTypeDef* adc_object){
	HAL_ADC_Start(adc_object);
	HAL_ADC_PollForConversion(adc_object, HAL_MAX_DELAY);
    uint32_t raw = HAL_ADC_GetValue(adc_object);
    HAL_ADC_Stop(adc_object);
    return raw;
}

#endif

