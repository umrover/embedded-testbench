
#include "servo.h"

// Constrain input value to bewteen maximum and minimum
int16_t constrain(int16_t val, int16_t min, int16_t max)
{

    if(val < min)
    {
        val = min;
    }
    else if (val > max) 
    {
        val = max;
    }

    return val;
}


Servo *new_servo(TIM_HandleTypeDef *_timer, uint32_t _channel, uint32_t *_out_channel)
{
	Servo *servo = (Servo*) malloc(sizeof(Servo));
	servo->timer = _timer;
	servo->channel = _channel;
	servo->out_channel = _out_channel;
    return servo;
}


void initialize_servo(Servo* servo, int16_t initial_angle)
{
	HAL_TIM_PWM_Start(servo->timer, servo->channel);
	set_servo_angle(servo, initial_angle);
}


void set_servo_angle(Servo *servo, int16_t angle)
{
	angle = constrain(angle, 180, 0);
	angle = constrain(angle, 0, 180);
    *(servo->out_channel) = (angle/18)+10;
}


