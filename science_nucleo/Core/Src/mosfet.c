//#ifdef MOSFET_ENABLE
#include "mosfet.h"

void enableRled(int enable){
    enablePin(enable, GPIOC,GPIO_PIN_8);
}
void enableGled(int enable){
    enablePin(enable, GPIOC,GPIO_PIN_6);
}
void enableBled(int enable){
    enablePin(enable, GPIOC,GPIO_PIN_7);
}
void enablesciUV(int enable){
    enablePin(enable, GPIOC,GPIO_PIN_10);
}
void enablesaUV(int enable){
    enablePin(enable, GPIOC,GPIO_PIN_11);
}
void enableWhiteled(int enable){
    enablePin(enable, GPIOC,GPIO_PIN_15);
}
void enablePerPump0(int enable){
	enablePin(enable, GPIOB,GPIO_PIN_9);
}
void enablePerPump1(int enable){
	enablePin(enable, GPIOB,GPIO_PIN_8);
}

void enablePin(int enable, GPIO_TypeDef *port, uint16_t pin){
    if (enable){
        HAL_GPIO_WritePin(port,pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
    }
}

//#endif

