#ifndef INC_PIN_H_
#define INC_PIN_H_


#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_it.h"


typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} Pin;


Pin *new_pin(GPIO_TypeDef *_port, uint16_t _pin);


#endif /* INC_PIN_H_ */
