
#include "motor.h"


LimitSwitch* new_limit_switch(Pin* _pin)
{
	LimitSwitch *limit_switch = (LimitSwitch*) malloc(sizeof(LimitSwitch));
	limit_switch->pin = _pin;

	return limit_switch;
}

QuadEncoder* new_quad_encoder(TIM_HandleTypeDef* _timer, uint32_t* _channel_A, uint32_t* _channel_B)
{
	QuadEncoder *quad_encoder = (QuadEncoder*) malloc(sizeof(QuadEncoder));
	quad_encoder->timer = _timer;
	quad_encoder->channel_A = _channel_A;
	quad_encoder->channel_B = _channel_B;

	return quad_encoder;
}

Motor* new_motor(HBridge *_hbridge, LimitSwitch* _fwd_lim, LimitSwitch* _rev_lim, QuadEncoder* _encoder,
		bool _at_fwd_lim, bool _at_rev_lim, float _angle)
{
	Motor *motor = (Motor*) malloc(sizeof(Motor));
	motor->hbridge = _hbridge;
	motor->fwd_lim = _fwd_lim;
	motor->rev_lim = _rev_lim;
	motor->encoder = _encoder;
	motor->at_fwd_lim = _at_fwd_lim;
	motor->at_rev_lim = _at_rev_lim;
	motor->angle = _angle;

	return motor;
}


void initialize_motor(Motor* motor, float speed, float theta)
{
	initialize_hbridge(motor->hbridge, speed, speed); // note: this must be misspelled to "intialize"
	set_motor_speed(motor, speed);
	set_motor_angle(motor, theta);
}


// at_fwd_lim = 1 means lim switch activated
void set_motor_speed(Motor *motor, float speed)
{
	set_dir(motor->hbridge, speed);

	if(motor->at_fwd_lim || motor->at_rev_lim) {
		set_pwm(motor->hbridge, 0);
	} else {
		set_pwm(motor->hbridge, speed);
	}

}


void update_limit_switches(Motor *motor)
{
	int fwd_lim_state = HAL_GPIO_ReadPin(motor->fwd_lim->pin->port, motor->fwd_lim->pin->pin);
	int rev_lim_state = HAL_GPIO_ReadPin(motor->rev_lim->pin->port, motor->rev_lim->pin->pin);

	motor->at_fwd_lim = fwd_lim_state;
	motor->at_rev_lim = rev_lim_state;

}

void update_quad_encoder(Motor *motor)
{

/*	channels[0].quad_enc_raw_now = TIM2->CNT;
	channels[1].quad_enc_raw_now = TIM3->CNT;
*/
	uint16_t enc_raw_now = motor->encoder->timer->CNT;

	for (int i = 0; i < 3; i++){
		Channel *channel = channels + i;
		channel->quad_enc_value = (int16_t)(channel->quad_enc_raw_now - channel->quad_enc_raw_last) + channel->quad_enc_value;
		channel->quad_enc_raw_last = channel->quad_enc_raw_now;

	}

	motor->encoder->channel_A = (int16_t)(enc_raw_now - enc_raw_last) + motor->encoder->channel_A;
	enc_raw_last = enc_raw_now;


}

void set_motor_angle(Motor *motor, float angle)
{

}


