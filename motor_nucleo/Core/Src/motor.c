
#include "motor.h"


void motor_periodic(TIM_HandleTypeDef *htim, TIM_HandleTypeDef *central_tim, Motor* motor[], int num_motors)
{
	if (htim == central_tim) {
		for(int i = 0; i < num_motors; ++i) {
			update_quad_encoder(motor[i]);
			update_limit_switches(motor[i]);
			update_pwm(motor[i]->hbridge);
		}
	}

}

LimitSwitch* new_limit_switch(Pin* _pin)
{
	LimitSwitch *limit_switch = (LimitSwitch*) malloc(sizeof(LimitSwitch));
	limit_switch->pin = _pin;

	return limit_switch;
}

QuadEncoder* new_quad_encoder(TIM_HandleTypeDef *_htim, TIM_TypeDef *_tim)
{
	QuadEncoder *quad_encoder = (QuadEncoder*) malloc(sizeof(QuadEncoder));
	quad_encoder->tim = _tim;
	quad_encoder->htim = _htim;

	return quad_encoder;
}

Motor* new_motor(HBridge *_hbridge, LimitSwitch* _fwd_lim, LimitSwitch* _rev_lim, QuadEncoder* _encoder,
		bool _at_fwd_lim, bool _at_rev_lim, int16_t _position)
{
	Motor *motor = (Motor*) malloc(sizeof(Motor));
	motor->hbridge = _hbridge;
	motor->fwd_lim = _fwd_lim;
	motor->rev_lim = _rev_lim;
	motor->encoder = _encoder;
	motor->at_fwd_lim = _at_fwd_lim;
	motor->at_rev_lim = _at_rev_lim;
	motor->position = _position;

	return motor;
}


void initialize_motor(Motor* motor, float speed, float theta)
{
	initialize_hbridge(motor->hbridge, speed, speed);
	set_motor_speed(motor, speed);
	set_motor_angle(motor, theta);
}

void initialize_quad_encoder(QuadEncoder* encoder)
{
	HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
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
	motor->encoder->raw = motor->encoder->tim->CNT;
	motor->position = (int16_t)(motor->encoder->raw - motor->encoder->prev_raw) + motor->position;
	motor->encoder->raw = motor->encoder->prev_raw;
}

void set_motor_angle(Motor *motor, float angle)
{



}


