////////////////////////////////
//      Thermistor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////


#include "thermistor.h"

Thermistor* thermistor;

uint32_t value[3];

float curr_temps[3] = { 100, 100, 100 }; // heater 2, 1, 0

Thermistor *new_thermistor(uint32_t channel0, uint32_t channel1, uint32_t channel2, ADC_HandleTypeDef hadc1) {
    Thermistor* therms = (Thermistor*) malloc(sizeof(Thermistor));
// string it in a struct doesn't work for some reason. I don't get it. It's above my paygrade.
//    therms-> adc = &hadc1;

    float temp_const_array[4][4] = {{3.3570420E-03, 2.5214848E-04, 3.3743283E-06, -6.4957311E-08},
                                  {3.3540170E-03, 2.5617244E-04, 2.1400943E-06, -7.2405219E-08},
                                  {3.3530481E-03, 2.5420230E-04, 1.1431163E-06, -6.9383563E-08},
                                  {3.3536166E-03, 2.5377200E-04, 8.5433271E-07, -8.7912262E-08}};

    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 4; ++j){
            therms->constant_array[i][j] = temp_const_array[i][j];
        }
    }

    HAL_ADC_Start_DMA(therms->adc,value,3);
    // NOTE make sure you set these to whatever your resistor values are
    therms->R1_vals[0] = 10000; therms->R1_vals[1] = 10000; therms->R1_vals[2] = 10000;

    therms->V1 = 3.3;

    therms->R25 = 10000;

    if (channel0 >= 0) {
        therms->adc_pins[0] = channel0;
    }
    if (channel1 >= 0) {
        therms->adc_pins[1] = channel1; 
    }
    if (channel2 >= 0) {
        therms->adc_pins[2] = channel2;
    }

    return therms;
}

void initialize_thermistor(Thermistor* thermistor) {
    return;
}

float get_thermistor_temperature(const uint8_t which_therm, Thermistor* thermistor, ADC_HandleTypeDef hadc1) {
    //Before reading voltage, enable only the selected channel
    uint32_t raw_data;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    value[0] = HAL_ADC_GetValue(&hadc1);
//    value[0] = HAL_ADC_GetValue(thermistor->adc);
// ADC_HandleTypeDef hadc1;
	raw_data = value[which_therm];

	// done to avoid sending infinity/nan
	if (raw_data >= 4095) raw_data = 4094;

    // Logic to get actual Voltage from 12 bit string
    // NOTE pretty sure it is 12 bit that's what HAL says in documentation, but could be wrong
    float curr_volt = (raw_data * thermistor->V1) / (float)4095; // 2^12 - 1= 4095 (12 bit string  )

    // Circuit math to get temperature from voltage
    float Rt = ((float)thermistor->R1_vals[which_therm] * curr_volt) / (thermistor->V1 - curr_volt);

    uint8_t const_set;
    if(Rt < 692600 && Rt >= 32770){
        const_set = 0;
    } else if (Rt < 32770 && Rt >= 3599){
        const_set = 1;
    } else if (Rt < 3599 && Rt >= 681.6){
        const_set = 2;
    } else if (Rt < 681.6 && Rt >= 187){
        const_set = 3;
    } else {
        // TODO error out cause OOB temp
    }
    float lnRt_over_R25 = log(Rt/thermistor->R25);

    float one_over_T = thermistor->constant_array[const_set][0] + (thermistor->constant_array[const_set][1] * lnRt_over_R25)
                     + (thermistor->constant_array[const_set][2] * lnRt_over_R25 * lnRt_over_R25)
                     + (thermistor->constant_array[const_set][3] * lnRt_over_R25 * lnRt_over_R25 * lnRt_over_R25);
    return (1 / one_over_T) - 272.15;
}


void deleteThermistors(Thermistor* thermistors){
    free(thermistors);
}

