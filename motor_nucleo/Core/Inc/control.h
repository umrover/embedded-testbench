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

} Gains;


Gains *new_gains(float kP_, float kI_, float kD_, float kF_);

float calculate_pid(Gains *gains, float target, float current, float dt);


#endif /* INC_CONTROL_H_ */
