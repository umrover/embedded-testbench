
#include "motor.h"

Motor *new_motor(HBridge *_hbridge, LimitSwitch *_fwd_lim, LimitSwitch *_bwd_lim, QuadEncoder *_encoder, Control *_control) {
    Motor *motor = (Motor *) malloc(sizeof(Motor));
    motor->hbridge = _hbridge;
    motor->forward_limit_switch = _fwd_lim;
    motor->backward_limit_switch = _bwd_lim;
    motor->encoder = _encoder;
    motor->control = _control;
    motor->desired_speed = 0;

    return motor;
}

void init_motor(Motor *motor, float speed) {
    init_hbridge(motor->hbridge, speed, speed);
    set_motor_speed(motor, speed);
}

// at_fwd_lim = 1 means lim switch activated
void set_motor_speed(Motor *motor, float speed) {
    motor->desired_speed = speed;
}

void update_motor_speed(Motor *motor) {
    // when speed is positive, motor goes from rev lim to fwd lim
    if (motor->forward_limit_switch->is_activated && (motor->desired_speed > 0)) {
        set_pwm(motor->hbridge, 0);
    } else if (motor->backward_limit_switch->is_activated && (motor->desired_speed < 0)) {
        set_pwm(motor->hbridge, 0);
    } else {
        set_pwm(motor->hbridge, fabs(motor->desired_speed));
    }

    set_dir(motor->hbridge, motor->desired_speed);
}

void move_motor_to_target(Motor *motor, int32_t target_counts, float dt) {
    // TODO need to test this blind implementation, may be some problems wrapping across counts boundaries
    float speed = calculate_pid(motor->control, target_counts, motor->encoder->counts, dt);
    set_motor_speed(motor, speed);
}


