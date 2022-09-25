
#include "servo.h"

// Constrain input value to bewteen maximum and minimum
uint16_t constrain(uint16_t val, uint16_t min, uint16_t max)
{

    // assert min <= max
    if(min > max)
    {
        uint16_t tmp = min;
        min = max;
        max = tmp;
    }

    // constrain val
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

// REQUIRES: timer is the timer and channel is the channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Servo object
Servo *new_servo(TIM_HandleTypeDef *_timer, uint32_t _channel, uint32_t *_out_channel)
{
	Servo *servo = (Servo*) malloc(sizeof(Servo));
	servo->timer = _timer;
	servo->channel = _channel;
	servo->out_channel = _out_channel;
    return servo;
}

// REQUIRES: servo is a Servo object and initial_angle is the initial angle
// MODIFIES: nothing
// EFFECTS: Initializes the servo by configuring the timer settings
void initialize_servo(Servo* servo, uint16_t initial_angle)
{
	HAL_TIM_PWM_Start(servo->timer, servo->channel);
	set_servo_angle(servo, initial_angle);
}

// REQUIRES: servo is a Servo object and angle is desired angle in degrees
// and 0 <= angle <= 180
// MODIFIES: nothing
// EFFECTS: Sets the servo angle to an absolute position
void set_servo_angle(Servo *servo, uint16_t angle)
{
    angle = constrain(angle, 0, 180);
    *(servo->out_channel) = (angle/18)+15;
}


