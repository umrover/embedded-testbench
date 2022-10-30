
#include "hbridge.h"


Pin* new_pin(GPIO_TypeDef *_port, uint16_t _pin)
{
	Pin* pin = (Pin*) malloc(sizeof(Pin));
	pin->port = _port;
	pin->pin = _pin;

	return pin;
}

HBridge* new_hbridge(TIM_HandleTypeDef *_timer, uint32_t _channel, uint32_t *_out_register, uint32_t _ARR, Pin* _fwd, Pin* _bwd)
{
    HBridge *hbr = (HBridge*) malloc(sizeof(HBridge));
    hbr->timer = _timer;
    hbr->channel = _channel;
    hbr->out_register = _out_register;
    hbr->ARR = _ARR;
    hbr->fwd = _fwd;
    hbr->bwd = _bwd;
    
    return hbr;
}


void initialize_hbridge(HBridge* hbridge, double duty_cycle, uint8_t direction)
{
    HAL_TIM_PWM_Start(hbridge->timer, hbridge->channel);
	set_pwm(hbridge, duty_cycle);
    set_dir(hbridge, direction);
}


void set_pwm(HBridge* hbridge, double duty_cycle)
{

    // validate input duty cycle
    if (duty_cycle < 0.0)
    {
        duty_cycle = 0.0;
    } 
    else if (duty_cycle > 1.0)
    {
        duty_cycle = 1.0;
    }

    *(hbridge->out_register) = duty_cycle * (double) hbridge->ARR;

}


void set_dir(HBridge* hbridge, int8_t direction)
{

    if (direction < 0)
    {
        HAL_GPIO_WritePin(hbridge->fwd->port, hbridge->fwd->pin, 0);
        HAL_GPIO_WritePin(hbridge->bwd->port, hbridge->bwd->pin, 1);
    }

    else if (direction > 0)
    {
        HAL_GPIO_WritePin(hbridge->fwd->port, hbridge->fwd->pin, 1);
        HAL_GPIO_WritePin(hbridge->bwd->port, hbridge->bwd->pin, 0);
    }

    else // direction == 0
    {
        set_pwm(hbridge, 0);
    }
    
}
