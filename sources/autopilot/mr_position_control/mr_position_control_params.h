/**
 * @file mr_position_control_params.h
 *
 * Parameters defined for the multirotor position control
 *
 */


#ifndef __MR_POSITION_CONTROL_PARAMS_H
#define __MR_POSITION_CONTROL_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"
	#include "../../uav_library/math/Vector3f.h"

	struct mr_position_control_parameters {
		float thr_min;
		float thr_max;
		float tilt_max;
		float land_speed;
		float land_tilt_max;

		float rc_scale_pitch;
		float rc_scale_roll;

		Vector3f pos_p;
		Vector3f vel_p;
		Vector3f vel_i;
		Vector3f vel_d;
		Vector3f vel_ff;
		Vector3f vel_max;
		Vector3f sp_offs_max;
	} mr_position_control_parameters;			/**< local copies of interesting parameters */

	struct mr_position_control_parameter_handles {
		param_t thr_min;
		param_t thr_max;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_ff;
		param_t tilt_max;
		param_t land_speed;
		param_t land_tilt_max;

		param_t rc_scale_pitch;
		param_t rc_scale_roll;
	} mr_position_control_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int mr_position_control_params_define ();
	int mr_position_control_params_update ();
	int mr_position_control_params_init ();

	/* global variables */
	// empty

#endif
