/**
 * @file thrust_pid.h
 *
 * Definition of thrust control PID interface.
 *
 */

#ifndef THRUST_PID_H_
#define THRUST_PID_H_

	#include <stdint.h>


	/* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error */
	#define THRUST_PID_MODE_DERIVATIV_CALC	0
	/* PID_MODE_DERIVATIV_CALC_NO_SP calculates discrete derivative from previous value, setpoint derivative is ignored */
	#define THRUST_PID_MODE_DERIVATIV_CALC_NO_SP	1

	typedef struct {
		float kp;
		float ki;
		float kd;
		float sp;
		float integral;
		float error_previous;
		float last_output;
		float limit_min;
		float limit_max;
		float dt_min;
		uint8_t mode;
	} thrust_pid_t;


	/* function prototypes */
	void thrust_pid_init(thrust_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max, uint8_t mode, float dt_min);
	int thrust_pid_set_parameters(thrust_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max);
	float thrust_pid_calculate(thrust_pid_t *pid, float sp, float val, float dt, float r22);
	void thrust_pid_set_integral(thrust_pid_t *pid, float i);

	/* global variables */
	// empty

#endif /* THRUST_PID_H_ */
