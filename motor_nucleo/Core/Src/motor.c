
#include "motor.h"

void motor_periodic(Motor *motor[], int num_motors) {
	for (int i = 0; i < num_motors; ++i) {
		if (motor[i]) {
			update_quad_encoder(motor[i]);
			update_motor_limits(motor[i]);
			update_motor_speed(motor[i]);
		}
	}
}

QuadEncoder *new_quad_encoder(TIM_HandleTypeDef *_htim, TIM_TypeDef *_tim, int16_t _PPR) {
    QuadEncoder *quad_encoder = (QuadEncoder *) malloc(sizeof(QuadEncoder));
    quad_encoder->tim = _tim;
    quad_encoder->htim = _htim;
    quad_encoder->PPR = _PPR;

    return quad_encoder;
}


Motor *
new_motor(HBridge *_hbridge, Pin *_fwd_lim, Pin *_bwd_lim, QuadEncoder *_encoder, Control *_control) {
    Motor *motor = (Motor *) malloc(sizeof(Motor));
    motor->hbridge = _hbridge;
    motor->forward_limit_switch_pin = _fwd_lim;
    motor->backward_limit_switch_pin = _bwd_lim;
    motor->encoder = _encoder;
    motor->control = _control;
    motor->at_fwd_lim = 0;
    motor->at_rev_lim = 0;
    motor->desired_speed = 0;

    return motor;
}


void initialize_motor(Motor *motor, float speed, float theta) {
    initialize_hbridge(motor->hbridge, speed, speed);
    set_motor_speed(motor, speed);
    set_motor_counts(motor, theta);
}


void initialize_quad_encoder(QuadEncoder *encoder) {
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
}


// at_fwd_lim = 1 means lim switch activated
void set_motor_speed(Motor *motor, float speed) {
    motor->desired_speed = speed;
}

void update_motor_speed(Motor *motor) {
    // when speed is positive, motor goes from rev lim to fwd lim
    if (motor->at_fwd_lim && (motor->desired_speed > 0)) {
        set_pwm(motor->hbridge, 0);
    } else if (motor->at_rev_lim && (motor->desired_speed < 0)) {
        set_pwm(motor->hbridge, 0);
    } else {
        set_pwm(motor->hbridge, fabs(motor->desired_speed));
    }

    set_dir(motor->hbridge, motor->desired_speed);
}


void update_motor_limits(Motor *motor) {
    if (motor->forward_limit_switch_pin) {
        int fwd_lim_state = HAL_GPIO_ReadPin(motor->forward_limit_switch_pin->port, motor->forward_limit_switch_pin->pin);
        motor->at_fwd_lim = fwd_lim_state;
    }
    if (motor->backward_limit_switch_pin) {
        int rev_lim_state = HAL_GPIO_ReadPin(motor->backward_limit_switch_pin->port, motor->backward_limit_switch_pin->pin);
        motor->at_rev_lim = rev_lim_state;
    }
}

void update_quad_encoder(Motor *motor) {
    motor->encoder->raw = motor->encoder->tim->CNT;

    motor->raw_counts = (int32_t)(((float) motor->encoder->raw / (float) motor->encoder->PPR) * 360.0);

    // ((n mod 360) + 360) mod 360  (int32_t)
    motor->counts = ((motor->raw_counts % 360) + 360) % 360;
}

void set_motor_counts(Motor *motor, int32_t counts) {
	motor->counts = 0;
	motor->raw_counts = 0;
}

void update_motor_counts(Motor *motor, int32_t counts, float dt) {
    // TODO need to test this blind implementation, may be some problems wrapping across counts boundaries
    float speed = calculate_pid(motor->control, counts, motor->counts, dt);
    set_motor_speed(motor, speed);

}


