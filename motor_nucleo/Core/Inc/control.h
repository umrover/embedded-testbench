#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_


#include <stdlib.h>
#include <stddef.h>
#include "stdbool.h"


typedef struct {

    // GAINS
    float kP;
    float kI;
    float kD;
    float kF;

    // MATH PLACEHOLDERS
    float last_error;
    float cum_integ;
    bool flag;

} Control;

/** PUBLIC FUNCTIONS **/

Control *new_control(float kP_, float kI_, float kD_, float kF_);

float calculate_pid(Control *control, float target, float current, float dt);

/** PRIVATE FUNCTIONS MAY BE IN SOURCE FILE ONLY **/

float signum(float num);

#endif /* INC_CONTROL_H_ */
