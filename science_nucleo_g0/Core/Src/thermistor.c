//////////////////////////////////////
//
//      Thermistor Nucleo Hardware Interface
//      Written By:
//      Nathan Richards
//      nricha@umich.edu
//
//////////////////////////////////////
//#define THERMISTOR_ENABLE

#include "thermistor.h"
#include "mosfet.h"



Thermistors* thermistors;
float curr_temps[3] = { 100, 100, 100 }; // heater 2, 1, 0

// Delay between temp of thermistor and the actual nichrome temp so be generous?
float max_temp = 65.0;

uint32_t value[3];

///////////////////
//
// Public Functions
//
///////////////////


Thermistors* new_thermistors(uint32_t channel0, uint32_t channel1, uint32_t channel2){
    Thermistors* therms = (Thermistors*) malloc(sizeof(Thermistors));

    float temp_const_array[4][4] = {{3.3570420E-03, 2.5214848E-04, 3.3743283E-06, -6.4957311E-08},
                                  {3.3540170E-03, 2.5617244E-04, 2.1400943E-06, -7.2405219E-08},
                                  {3.3530481E-03, 2.5420230E-04, 1.1431163E-06, -6.9383563E-08},
                                  {3.3536166E-03, 2.5377200E-04, 8.5433271E-07, -8.7912262E-08}};

    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 4; ++j){
            therms->constant_array[i][j] = temp_const_array[i][j];
        }
    }
    HAL_ADC_Start_DMA(&hadc1,value,3);
    // NOTE make sure you set these to whatever your resistor values are
    therms->R1_vals[0] = 10000; therms->R1_vals[1] = 10000; therms->R1_vals[2] = 10000;

    therms->V1 = 3.3;

    therms->R25 = 10000;

    therms->adc_pins[0] = channel0; therms->adc_pins[1] = channel1; therms->adc_pins[2] = channel2;

    return therms;
}




float get_temp(const uint8_t which_therm, const Thermistors* therms){
    
    //Before reading voltage, enable only the selected channel

	//uint32_t raw_data = _readVoltage(therms->adc_pins[which_therm]);
	uint32_t raw_data = value[which_therm];

	// done to avoid sending infinity/nan
	if (raw_data >= 4095) raw_data = 4094;

    // Logic to get actual Voltage from 12 bit string
    // NOTE pretty sure it is 12 bit that's what HAL says in documentation, but could be wrong
    float curr_volt = (raw_data * therms->V1) / (float)4095; // 2^12 - 1= 4095 (12 bit string  )

    // Circuit math to get temperature from voltage
    float Rt = ((float)therms->R1_vals[which_therm] * curr_volt) / (therms->V1 - curr_volt);

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
    float lnRt_over_R25 = log(Rt/therms->R25);

    float one_over_T = therms->constant_array[const_set][0] + (therms->constant_array[const_set][1] * lnRt_over_R25)
                     + (therms->constant_array[const_set][2] * lnRt_over_R25 * lnRt_over_R25)
                     + (therms->constant_array[const_set][3] * lnRt_over_R25 * lnRt_over_R25 * lnRt_over_R25);
    return (1 / one_over_T) - 272.15;
}


void deleteThermistors(Thermistors* thermistors){
    free(thermistors);
}

void send_thermistor_data(Thermistors* therms, UART_HandleTypeDef* huart){
  HAL_ADC_Start_DMA(&hadc1,value,3);
  for(int t = 0; t < 3; t++){
    curr_temps[t] = get_temp(t, therms);

  }

  char string[50] = "";

  sprintf((char *)string, "$THERMISTOR,%f,%f,%f,\n", curr_temps[THERMISTOR_OF_HEATER_0],
		  curr_temps[THERMISTOR_OF_HEATER_1], curr_temps[THERMISTOR_OF_HEATER_2]);
  //HAL_UART_Transmit(huart, (uint8_t *)string, sizeof(string), 15);
  HAL_Delay(300);
  HAL_UART_Transmit_IT(huart, (uint8_t *)string, sizeof(string));
  HAL_Delay(15);
  // Delay before Clearing flags so beaglebone can successfully read the
//  HAL_Delay(100);


}

///////////////////
//
// Private Functions
//
///////////////////
