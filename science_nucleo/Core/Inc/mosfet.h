#ifndef MOSFET_H_
#define MOSFET_H_

#include "stm32f3xx_hal.h"


//Public Calls
//Could also be privater with a larger overall enable that takes a string or something 
//Enable/disable the specific device
void enableRled(int enable);
void enableGled(int enable);
void enableBled(int enable);
void enableHeater(int enable);
void enablesciUV(int enable);
void enablesaUV(int enable);
void enableWhiteled(int enable);



// Private Interface
// Enable or disable a certain pin
void enablePin(int enable, int mosfetdevice);



#endif