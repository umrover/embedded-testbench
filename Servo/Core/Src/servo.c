#include "servo.h"

Servo* new_servo(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t *output) {

	Servo* servo = (Servo*) malloc(sizeof(Servo));
	servo->timer = timer;
	servo->channel = channel;
	servo->output = output;
	return servo;

}


void set_servo_angle(Servo* servo, double angle) {
	*(servo->output) = (uint32_t) (((5.0/9) * angle) + 150.0);//(uint32_t) (2.0/1.8 * angle);
}

void initialize_servo(Servo* servo, double angle) {
	HAL_TIM_PWM_Start(servo->timer, servo->channel);
	set_servo_angle(servo, angle);
}
