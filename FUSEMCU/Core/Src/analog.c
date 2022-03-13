#ifndef INC_ANALOG_H_

#include "analog.h"
#include "main.h"

// Current Sensor: ACS722LLCTR-10AU-T

// Pin and Port arrays for the analog devices
// Voltage and current pins and ports
uint16_t pin_array[10] = {
	Voltage_Select_Pin,					// 0
	Voltage_SelectA3_Pin,				// 1
	Voltage_SelectA4_Pin,				// 2
	Voltage_SelectA5_Pin,				// 3
	Voltage_Enable_Pin,					// 4
	Current_SelectA8_Pin				// 5
	Current_SelectB15_Pin,				// 6
	Current_SelectB14_Pin,				// 7
	Current_Select_Pin,					// 8
	Current_Enable_Pin					// 9
};

GPIO_TypeDef* port_array[10] = {
	Voltage_Select_GPIO_Port,			// 0
	Voltage_SelectA3_GPIO_Port,			// 1
	Voltage_SelectA4_GPIO_Port,			// 2
	Voltage_SelectA5_GPIO_Port,			// 3
	Voltage_Enable_GPIO_Port,			// 4
	Current_SelectA8_GPIO_Port,			// 5
	Current_SelectB15_GPIO_Port,		// 6
	Current_SelectB14_GPIO_Port,		// 7
	Current_Select_GPIO_Port,			// 8
	Current_Enable_GPIO_Port			// 9
};

// EFFECTS: Create a new Analog object and returns pointer to object.
// TODO
Analog* new_analog(ADC_HandleTypeDef* in, uint8_t* select_pins, size_t sz){

	// Create a new struct
	Analog* Analog_object = (Analog*) malloc(sizeof(Analog));

	Analog_object->in = in;
	Analog_object->select_pins[0] = select_pins[0];
	Analog_object->select_pins[1] = select_pins[1];
	Analog_object->select_pins[2] = select_pins[2];
	Analog_object->select_pins[3] = select_pins[3];

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

