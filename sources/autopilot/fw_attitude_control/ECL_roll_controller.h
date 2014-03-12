/**
 * @file ecl_roll_controller.h
 * Definition of a simple orthogonal roll PID controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef ECL_ROLL_CONTROLLER_H
#define ECL_ROLL_CONTROLLER_H

	#include <stdint.h>
	#include "../../uav_library/common.h"


	float ECL_roll_controller_control(float roll_setpoint, float roll, float roll_rate, float scaler, bool_t lock_integrator, float airspeed_min, float airspeed_max, float airspeed);
	void ECL_roll_controller_init ();

	void ECL_roll_controller_reset_integrator();

	void ECL_roll_controller_set_time_constant(float time_constant);
	void ECL_roll_controller_set_k_p(float k_p);
	void ECL_roll_controller_set_k_i(float k_i);
	void ECL_roll_controller_set_k_d(float k_d);
	void ECL_roll_controller_set_integrator_max(float max);
	void ECL_roll_controller_set_max_rate(float max_rate);

	float ECL_roll_controller_get_rate_error();
	float ECL_roll_controller_get_desired_rate();

#endif // ECL_ROLL_CONTROLLER_H
