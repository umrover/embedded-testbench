#include "control.h"


Gains* new_gains(float kP_, float kI_, float kD_, float kF_)
{
	Gains* gains = (Gains*) malloc(sizeof(Gains));

	gains->kP = kP_;
	gains->kI = kI_;
	gains->kD = kD_;
	gains->kF = kF_;

	gains->flag = 1;
	gains->last_error = 0.0;
	gains->cum_integ = 0.0;

	return gains;
}


float signum(float num)
{
	if(num < 0)
	{
		return -1.0;
	}
	if(num > 0)
	{
		return 1.0;
	}
	return 0.0;
}


float calculate_pid(Gains* gains, float target, float current, float dt)
{
	if (gains->flag)
	{
		gains->last_error = target - current;
		gains->flag = 0;
	}

	float error = target - current;

	gains->cum_integ += error * dt;
	float diff = (error - gains->last_error) / dt;

	float output = gains->kP * error + gains->kI * gains->cum_integ + gains->kD * diff + signum(error) * gains->kF;

	gains->last_error = error;
	return output;
}


