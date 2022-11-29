#include "control.h"


Control *new_control(float kP_, float kI_, float kD_, float kF_) {
    Control *control = (Control *) malloc(sizeof(Control));

    control->kP = kP_;
    control->kI = kI_;
    control->kD = kD_;
    control->kF = kF_;

    control->flag = 1;
    control->last_error = 0.0;
    control->cum_integ = 0.0;

    return control;
}

float calculate_pid(Control *control, float target, float current, float dt) {
    if (control->flag) {
        control->last_error = target - current;
        control->flag = 0;
    }

    float error = target - current;

    control->cum_integ += error * dt;
    float diff = (error - control->last_error) / dt;

    float output = control->kP * error + control->kI * control->cum_integ + control->kD * diff + signum(error) * control->kF;

    control->last_error = error;
    return output;
}

float signum(float num) {
    if (num < 0) {
        return -1.0;
    }
    if (num > 0) {
        return 1.0;
    }
    return 0.0;
}
