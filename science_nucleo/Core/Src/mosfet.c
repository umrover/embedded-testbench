#include "mosfet.h"

void enableRled(int enable){
    enablePin(enable, GPIOA,2);
}
void enableGled(int enable){
    enablePin(enable, GPIOA,2);
}
void enableBled(int enable){
    enablePin(enable, GPIOA,2);
}
void enableHeater(int enable){
    enablePin(enable, GPIOA,2);
}
void enablesciUV(int enable){
    enablePin(enable, GPIOA,2);
}
void enablesaUV(int enable){
    enablePin(enable, GPIOA,2);
}
void enableWhiteled(int enable){
    enablePin(enable, GPIOA,2);
}

void enablePin(int enable, GPIO_TypeDef *port,uint16_t pin){
    if (enable){
        HAL_GPIO_WritePin(port,pin GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
    }
}