/**
 * @file pid.c
 *
 * Implementation of generic PID control interface.
 *
 * @author Laurens Mackay <mackayl@student.ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Martin Rutschmann <rutmarti@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include "pid.h"
#include "../math/limits.h"
#include <math.h>

#define SIGMA 0.000001f

void pid_init(PID_t *pid, float kp, float ki, float kd, float intmax,
		       float limit, uint8_t mode, float dt_min)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->intmax = intmax;
	pid->limit = limit;
	pid->mode = mode;
	pid->dt_min = dt_min;
	pid->count = 0.0f;
	pid->saturated = 0.0f;
	pid->last_output = 0.0f;
	pid->sp = 0.0f;
	pid->error_previous = 0.0f;
	pid->integral = 0.0f;
}

int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float intmax, float limit)
{
	int ret = 0;

	if (check_finite(kp)) {
		pid->kp = kp;

	} else {
		ret = 1;
	}

	if (check_finite(ki)) {
		pid->ki = ki;

	} else {
		ret = 1;
	}

	if (check_finite(kd)) {
		pid->kd = kd;

	} else {
		ret = 1;
	}

	if (check_finite(intmax)) {
		pid->intmax = intmax;

	}  else {
		ret = 1;
	}

	if (check_finite(limit)) {
		pid->limit = limit;

	}  else {
		ret = 1;
	}

	return ret;
}

//void pid_set(PID_t *pid, float sp)
//{
//	pid->sp = sp;
//	pid->error_previous = 0;
//	pid->integral = 0;
//}

/**
 *
 * @param pid
 * @param val
 * @param dt
 * @return
 */
float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
	/*  error = setpoint - actual_position
	 integral = integral + (error*dt)
	 derivative = (error - previous_error)/dt
	 output = (Kp*error) + (Ki*integral) + (Kd*derivative)
	 previous_error = error
	 wait(dt)
	 goto start
	 */

	if (!check_finite(sp) || !check_finite(val) || !check_finite(val_dot) || !check_finite(dt)) {
		return pid->last_output;
	}

	float i, d;
	pid->sp = sp;

	// Calculated current error value
	float error = pid->sp - val;

	// Calculate or measured current error derivative
	if (pid->mode == PID_MODE_DERIVATIV_CALC) {
		d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = error;

	} else if (pid->mode == PID_MODE_DERIVATIV_CALC_NO_SP) {
		d = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = -val;

	} else if (pid->mode == PID_MODE_DERIVATIV_SET) {
		d = -val_dot;

	} else {
		d = 0.0f;
	}

	if (!check_finite(d)) {
		d = 0.0f;
	}

	if (pid->ki > 0.0f) {
		// Calculate the error integral and check for saturation
		i = pid->integral + (error * dt);

		if ((pid->limit > SIGMA && (fabsf((error * pid->kp) + (i * pid->ki) + (d * pid->kd)) > pid->limit)) ||
		    fabsf(i) > pid->intmax) {
			i = pid->integral;		// If saturated then do not update integral value
			pid->saturated = 1;

		} else {
			if (!check_finite(i)) {
				i = 0.0f;
			}

			pid->integral = i;
			pid->saturated = 0;
		}

	} else {
		i = 0.0f;
		pid->saturated = 0;
	}

	// Calculate the output.  Limit output magnitude to pid->limit
	float output = (error * pid->kp) + (i * pid->ki) + (d * pid->kd);

	if (check_finite(output)) {
		if (pid->limit > SIGMA) {
			if (output > pid->limit) {
				output = pid->limit;

			} else if (output < -pid->limit) {
				output = -pid->limit;
			}
		}

		pid->last_output = output;
	}

	return pid->last_output;
}


void pid_reset_integral(PID_t *pid)
{
	pid->integral = 0;
}
