//#ifdef MOSFET_ENABLE
#include "mosfet.h"

void enableRled(int enable){
    enablePin(enable, GPIOC,7);
}
void enableGled(int enable){
    enablePin(enable, GPIOA,9);
}
void enableBled(int enable){
    enablePin(enable, GPIOA,8);
}
void enablesciUV(int enable){
    enablePin(enable, GPIOC,9);
}
void enablesaUV(int enable){
    enablePin(enable, GPIOA,10);
}
void enableWhiteled(int enable){
    enablePin(enable, GPIOA,13);
}

void enablePin(int enable, GPIO_TypeDef *port, uint16_t pin){
    if (enable){
        HAL_GPIO_WritePin(port,pin, GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
    }
}

//#endif
