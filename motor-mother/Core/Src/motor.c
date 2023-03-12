
#include "motor.h"

Motor *new_motor(bool _valid, HBridge *_hbridge, LimitSwitch *_fwd_lim, LimitSwitch *_bwd_lim, QuadEncoder *_encoder, AbsEncoder *_abs_encoder, ClosedLoopControl *_control) {
    Motor *motor = (Motor *) malloc(sizeof(Motor));
    motor->valid = _valid;
    motor->hbridge = _hbridge;
    motor->forward_limit_switch = _fwd_lim;
    motor->backward_limit_switch = _bwd_lim;
    motor->encoder = _encoder;
    motor->abs_encoder = _abs_encoder;

    motor->control = _control;
    motor->using_open_loop_control = true;
    motor->output_pwm = 0;
    motor->max_pwm = 1;
    motor->desired_speed = 0;
    motor->desired_counts = 0;
    motor->limit_enabled = true;// TODO this should be true... testing

    return motor;
}

void init_motor(Motor *motor, float speed) {
    init_hbridge(motor->hbridge, speed, speed);
    set_motor_speed(motor, speed);
}

void update_motor_target(Motor *motor) {
	if (motor->using_open_loop_control) {
		set_motor_speed(motor, motor->desired_speed);
	}
	else {
        move_motor_to_target(motor);
    }
}

// at_fwd_lim = 1 means lim switch activated
void set_motor_speed(Motor *motor, float speed) {
	motor->output_pwm = speed * motor->max_pwm;
}

void update_motor_speed(Motor *motor) {
    // when speed is positive, motor goes from rev lim to fwd lim
	if (motor->limit_enabled) {
		if (motor->forward_limit_switch && motor->forward_limit_switch->is_activated && (motor->output_pwm > 0.0f)) {
			change_hbridge_pwm(motor->hbridge, 0.0f);
		} else if (motor->backward_limit_switch && motor->backward_limit_switch->is_activated && (motor->output_pwm < 0.0f)) {
			change_hbridge_pwm(motor->hbridge, 0.0f);
		} else {
			change_hbridge_pwm(motor->hbridge, fabsf(motor->output_pwm));
		}
		change_hbridge_dir_val(motor->hbridge, motor->output_pwm > 0.0f);
	}
	else {
		change_hbridge_pwm(motor->hbridge, fabsf(motor->output_pwm));
		change_hbridge_dir_val(motor->hbridge, motor->output_pwm > 0.0f);
	}

}

void move_motor_to_target(Motor *motor) {
    // TODO need to test this blind implementation, may be some problems wrapping across counts boundaries
    float speed = calculate_pid(motor->control, motor->desired_counts, motor->encoder->counts);
    set_motor_speed(motor, speed);
}

void refresh_motor_absolute_encoder_value(Motor *motor) {
	if (motor->abs_encoder->valid) {
		refresh_angle_radians(motor->abs_encoder);
	}
}


