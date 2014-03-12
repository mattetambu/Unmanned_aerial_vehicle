/**
 * @file ecl_yaw_controller.h
 * Definition of a simple orthogonal coordinated turn yaw PID controller.
 *
 */

#ifndef ECL_YAW_CONTROLLER_H
#define ECL_YAW_CONTROLLER_H

	#include <stdint.h>
	#include "../../uav_library/common.h"


	float ECL_yaw_controller_control(float roll, float yaw_rate, float accel_y, float scaler, bool_t lock_integrator, float airspeed_min, float airspeed_max, float aspeed);
	void ECL_yaw_controller_init ();

	void ECL_yaw_controller_reset_integrator();

	void ECL_yaw_controller_set_k_side(float k_a);
	void ECL_yaw_controller_set_k_i(float k_i);
	void ECL_yaw_controller_set_k_d(float k_d);
	void ECL_yaw_controller_set_k_roll_ff(float k_roll_ff);
	void ECL_yaw_controller_set_integrator_max(float max);

#endif // ECL_YAW_CONTROLLER_H
