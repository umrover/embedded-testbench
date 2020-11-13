//#ifdef MOSFET_ENABLE
#include "mosfet.h"

void enableRled(int enable){
    enablePin(enable, GPIOC,GPIO_PIN_7);
}
void enableGled(int enable){
    enablePin(enable, GPIOA,GPIO_PIN_9);
}
void enableBled(int enable){
    enablePin(enable, GPIOA,GPIO_PIN_8);
}
void enablesciUV(int enable){
    enablePin(enable, GPIOC,GPIO_PIN_9);
}
void enablesaUV(int enable){
    enablePin(enable, GPIOA,GPIO_PIN_10);
}
void enableWhiteled(int enable){
    enablePin(enable, GPIOA,GPIO_PIN_13);
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
