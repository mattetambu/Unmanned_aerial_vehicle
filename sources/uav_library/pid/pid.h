/**
 * @file pid.h
 *
 * Definition of generic PID control interface.
 *
 * @author Laurens Mackay <mackayl@student.ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Martin Rutschmann <rutmarti@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#ifndef PID_H_
#define PID_H_

	#include <stdint.h>

	/* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error
	 * val_dot in pid_calculate() will be ignored */
	#define PID_MODE_DERIVATIV_CALC	0
	/* PID_MODE_DERIVATIV_CALC_NO_SP calculates discrete derivative from previous value, setpoint derivative is ignored
	 * val_dot in pid_calculate() will be ignored */
	#define PID_MODE_DERIVATIV_CALC_NO_SP	1
	/* Use PID_MODE_DERIVATIV_SET if you have the derivative already (Gyros, Kalman) */
	#define PID_MODE_DERIVATIV_SET	2
	// Use PID_MODE_DERIVATIV_NONE for a PI controller (vs PID)
	#define PID_MODE_DERIVATIV_NONE 9

	typedef struct {
		float kp;
		float ki;
		float kd;
		float intmax;
		float sp;
		float integral;
		float error_previous;
		float last_output;
		float limit;
		float dt_min;
		uint8_t mode;
		uint8_t count;
		uint8_t saturated;
	} PID_t;

	void pid_init(PID_t *pid, float kp, float ki, float kd, float intmax, float limit, uint8_t mode, float dt_min);
	int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float intmax, float limit);
	//void pid_set(PID_t *pid, float sp);
	float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt);

	void pid_reset_integral(PID_t *pid);

#endif /* PID_H_ */
