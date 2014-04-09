/**
 * @file thrust_pid.c
 *
 * Implementation of thrust control PID.
 *
 */

#include "thrust_pid.h"
#include <math.h>
#include "../../uav_library/math/limits.h"


void thrust_pid_init(thrust_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max, uint8_t mode, float dt_min)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->limit_min = limit_min;
	pid->limit_max = limit_max;
	pid->mode = mode;
	pid->dt_min = dt_min;
	pid->last_output = 0.0f;
	pid->sp = 0.0f;
	pid->error_previous = 0.0f;
	pid->integral = 0.0f;
}

int thrust_pid_set_parameters(thrust_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max)
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

	if (check_finite(limit_min)) {
		pid->limit_min = limit_min;

	}  else {
		ret = 1;
	}

	if (check_finite(limit_max)) {
		pid->limit_max = limit_max;

	}  else {
		ret = 1;
	}

	return ret;
}

float thrust_pid_calculate(thrust_pid_t *pid, float sp, float val, float dt, float r22)
{
	/* Alternative integral component calculation
	 *
	 * start:
	 * error = setpoint - current_value
	 * integral = integral + (Ki * error * dt)
	 * derivative = (error - previous_error) / dt
	 * previous_error = error
	 * output = (Kp * error) + integral + (Kd * derivative)
	 * wait(dt)
	 * goto start
	 */

	if (!check_finite(sp) || !check_finite(val) || !check_finite(dt)) {
		return pid->last_output;
	}

	float i, d;
	pid->sp = sp;

	// Calculated current error value
	float error = pid->sp - val;

	// Calculate or measured current error derivative
	if (pid->mode == THRUST_PID_MODE_DERIVATIV_CALC) {
		d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = error;

	} else if (pid->mode == THRUST_PID_MODE_DERIVATIV_CALC_NO_SP) {
		d = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = -val;

	} else {
		d = 0.0f;
	}

	if (!check_finite(d)) {
		d = 0.0f;
	}

	/* calculate the error integral */
	i = pid->integral + (pid->ki * error * dt);

	/* attitude-thrust compensation
	 * r22 is (2, 2) componet of rotation matrix for current attitude */
	float att_comp;

	if (r22 > 0.8f)
		att_comp = 1.0f / r22;
	else if (r22 > 0.0f)
		att_comp = ((1.0f / 0.8f - 1.0f) / 0.8f) * r22 + 1.0f;
	else
		att_comp = 1.0f;

	/* calculate output */
	float output = ((error * pid->kp) + i + (d * pid->kd)) * att_comp;

	/* check for saturation */
	if (output < pid->limit_min || output > pid->limit_max) {
		/* saturated, recalculate output with old integral */
		output = (error * pid->kp) + pid->integral + (d * pid->kd);

	} else {
		if (check_finite(i)) {
			pid->integral = i;
		}
	}

	if (check_finite(output)) {
		if (output > pid->limit_max) {
			output = pid->limit_max;

		} else if (output < pid->limit_min) {
			output = pid->limit_min;
		}

		pid->last_output = output;
	}

	return pid->last_output;
}

void thrust_pid_set_integral(thrust_pid_t *pid, float i)
{
	pid->integral = i;
}
