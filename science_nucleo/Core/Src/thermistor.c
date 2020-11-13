//////////////////////////////////////
//
//      Thermistor Nucleo Hardware Interface
//      Written By:
//      Nathan Richards
//      nricha@umich.edu
//
//////////////////////////////////////

#ifdef THERMISTOR_ENABLE

#include "thermistor.h"


///////////////////
//
// Public Functions
//
///////////////////


Thermistors* newThermistors(ADC_HandleTypeDef* adc0, ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2){
    Thermistors* therms = (Thermistors*) malloc(sizeof(Thermistors));

    float tempConstArray[4][4] = {{3.3570420E-03, 2.5214848E-04, 3.3743283E-06, -6.4957311E-08},
                                  {3.3540170E-03, 2.5617244E-04, 2.1400943E-06, -7.2405219E-08},
                                  {3.3530481E-03, 2.5420230E-04, 1.1431163E-06, -6.9383563E-08},
                                  {3.3536166E-03, 2.5377200E-04, 8.5433271E-07, -8.7912262E-08}};

    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 4; ++j){
            therms->constantArray[i][j] = tempConstArray[i][j];
        }
    }

    // NOTE make sure you set these to whatever your resistor values are
    therms->R1vals[0] = 9820; therms->R1vals[1] = 10020; therms->R1vals[2] = 9830;

    // FIXME unsure if this will be correct value.  This is source Voltage
    // TODO might be able to set V1 to Vref+ for more accuracy
    therms->V1 = 5;

    therms->R25 = 10000;

    therms->adcPins[0] = adc0; therms->adcPins[1] = adc1; therms->adcPins[2] = adc2;

    return therms;
}




float getTemp(const uint8_t whichTherm, const Thermistors* therms){
    
    uint16_t rawData = readVoltage(therms->adcPins[whichTherm]);

    // Logic to get actual Voltage from 12 bit string
    // NOTE pretty sure it is 12 bit that's what HAL says in documentation, but could be wrong
    float currVolt = (float)rawData * therms->V1 / (float)4095; // 2^12 - 1= 4095 (12 bit string btw)

    // Circuit math to get temperature from voltage
    float Rt = (((float)therms->R1vals[whichTherm] * therms->V1) / currVolt) - therms->R1vals[whichTherm];

    uint8_t constSet;
    if(Rt < 69200 && Rt >= 32770){
        constSet = 0;
    } else if (Rt < 32770 && Rt >= 3599){
        constSet = 1;
    } else if (Rt < 3599 && Rt >= 681.6){
        constSet = 2;
    } else if (Rt < 681.6 && Rt >= 187){
        constSet = 3;
    } else {
        // TODO error out cause OOB temp
    }

    float lnRtoverR25 = log(Rt/therms->R25);

    float oneOverT = therms->constantArray[constSet][0] + (therms->constantArray[constSet][1] * lnRtoverR25)
                     + (therms->constantArray[constSet][2] * lnRtoverR25 * lnRtoverR25)
                     + (therms->constantArray[constSet][3] * lnRtoverR25 * lnRtoverR25 * lnRtoverR25);

    return (1 / oneOverT) - 272.15;
}


void deleteThermistors(Thermistors* thermistors){
    free(thermistors);
}


///////////////////
//
// Private Functions
//
///////////////////


uint16_t readVoltage(ADC_HandleTypeDef* adcObject){
    HAL_ADC_Start(adcObject);
    HAL_ADC_PollForConversion(adcObject, HAL_MAX_DELAY);
    uint16_t raw = HAL_ADC_GetValue(adcObject);
    HAL_ADC_Stop(adcObject);
    return raw;
}

#endif
