/**
 * @file ecl_pitch_controller.h
 * Definition of a simple orthogonal pitch PID controller.
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

#ifndef ECL_PITCH_CONTROLLER_H
#define ECL_PITCH_CONTROLLER_H

	#include <stdint.h>
	#include "../../uav_library/common.h"

	float ECL_pitch_controller_control(float pitch_setpoint, float pitch, float pitch_rate, float roll, float scaler, bool_t lock_integrator, float airspeed_min, float airspeed_max, float airspeed);
	void ECL_pitch_controller_init ();

	void ECL_pitch_controller_reset_integrator();

	void ECL_pitch_controller_set_time_constant(float time_constant);
	void ECL_pitch_controller_set_k_p(float k_p);
	void ECL_pitch_controller_set_k_i(float k_i);
	void ECL_pitch_controller_set_k_d(float k_d);
	void ECL_pitch_controller_set_integrator_max(float max);
	void ECL_pitch_controller_set_max_rate_pos(float max_rate_pos);
	void ECL_pitch_controller_set_max_rate_neg(float max_rate_neg);
	void ECL_pitch_controller_set_roll_ff(float roll_ff);

	float ECL_pitch_controller_get_rate_error();
	float ECL_pitch_controller_get_desired_rate();

#endif // ECL_PITCH_CONTROLLER_H
