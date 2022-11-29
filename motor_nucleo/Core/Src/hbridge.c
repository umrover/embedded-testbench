
#include "hbridge.h"


HBridge *new_hbridge(TIM_HandleTypeDef *_timer, uint32_t _channel, uint32_t *_out_register, uint32_t _ARR, Pin *_fwd,
                     Pin *_bwd) {
    HBridge *hbr = (HBridge *) malloc(sizeof(HBridge));
    hbr->timer = _timer;
    hbr->channel = _channel;
    hbr->out_register = _out_register;
    hbr->ARR = _ARR;
    hbr->fwd = _fwd;
    hbr->bwd = _bwd;
    hbr->target_duty_cycle = 0;

    return hbr;
}


void initialize_hbridge(HBridge *hbridge, float duty_cycle, uint8_t direction) {
    HAL_TIM_PWM_Start(hbridge->timer, hbridge->channel);
    set_pwm(hbridge, duty_cycle);
    set_dir(hbridge, direction);
}


void set_pwm(HBridge *hbridge, float duty_cycle) {

    // validate input duty cycle
    if (duty_cycle < 0.0) {
        duty_cycle = 0.0;
    } else if (duty_cycle > 1.0) {
        duty_cycle = 1.0;
    }

    hbridge->target_duty_cycle = duty_cycle * (double) hbridge->ARR;

    *(hbridge->out_register) = hbridge->target_duty_cycle;

}


void set_dir(HBridge *hbridge, float speed) {

    if (speed < 0) {
        HAL_GPIO_WritePin(hbridge->fwd->port, hbridge->fwd->pin, 0);
        HAL_GPIO_WritePin(hbridge->bwd->port, hbridge->bwd->pin, 1);
    } else {
        HAL_GPIO_WritePin(hbridge->fwd->port, hbridge->fwd->pin, 1);
        HAL_GPIO_WritePin(hbridge->bwd->port, hbridge->bwd->pin, 0);
    }

}
