
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
Servo *new_servo(TIM_HandleTypeDef *_timer, uint8_t _channel)
{
    Servo *servo(_timer, _channel);
    return servo;
}

// REQUIRES: servo is a Servo object
// MODIFIES: nothing
// EFFECTS: Initializes the servo by configuring the timer settings
void initialize_servo(Servo* servo)
{
    
}

// REQUIRES: servo is a Servo object and angle is desired angle in degrees
// and 0 <= angle <= 180
// MODIFIES: nothing
// EFFECTS: Sets the servo angle to an absolute position
uint32_t set_servo_angle(Servo *servo, uint16_t angle) 
{
    angle = constrain(angle);

}


