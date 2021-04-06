#ifndef MOSFET_H_
#define MOSFET_H_

#include "stm32f3xx_hal.h"


//Public Calls
//Could also be privater with a larger overall enable that takes a string or something 
//Enable/disable the specific device
void enableRled(int enable);
void enableGled(int enable);
void enableBled(int enable);
void enablesciUV(int enable);
void enablesaUV(int enable);
void enableWhiteled(int enable);
void enablePerPump0(int enable);
void enablePerPump1(int enable);


// Private Interface
// Enable or disable a certain pin
void enablePin(int enable, GPIO_TypeDef *port, uint16_t pin);

//GPIO_TypeDef *device_ports[6] = {
//		GPIOC,
//		GPIOC,
//		GPIOC,
//		GPIOC,
//		GPIOC,
//		GPIOC
//};
//
//int device_pins[6] = {
//		0,
//		0,
//		0,
//		GPIO_PIN_10,
//		GPIO_PIN_11,
//		GPIO_PIN_15
//};
//void setPin(int set, int device);



#endif

