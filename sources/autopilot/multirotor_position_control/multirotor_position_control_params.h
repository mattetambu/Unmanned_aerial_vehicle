/**
 * @file multirotor_position_control_params.h
 *
 * Parameters defined for the multirotor position control
 *
 */


#ifndef __MULTIROTOR_POSITION_CONTROL_PARAMS_H
#define __MULTIROTOR_POSITION_CONTROL_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct multirotor_position_control_parameters {
		float takeoff_alt;
		float takeoff_gap;
		float thr_min;
		float thr_max;
		float z_p;
		float z_d;
		float z_vel_p;
		float z_vel_i;
		float z_vel_d;
		float z_vel_max;
		float xy_p;
		float xy_d;
		float xy_vel_p;
		float xy_vel_i;
		float xy_vel_d;
		float xy_vel_max;
		float tilt_max;

		float rc_scale_pitch;
		float rc_scale_roll;
		float rc_scale_yaw;
	} multirotor_position_control_parameters;			/**< local copies of interesting parameters */

	struct multirotor_position_control_parameter_handles {
		param_t takeoff_alt;
		param_t takeoff_gap;
		param_t thr_min;
		param_t thr_max;
		param_t z_p;
		param_t z_d;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max;
		param_t xy_p;
		param_t xy_d;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t tilt_max;

		param_t rc_scale_pitch;
		param_t rc_scale_roll;
		param_t rc_scale_yaw;
	} multirotor_position_control_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int multirotor_position_control_params_define ();
	int multirotor_position_control_params_update ();
	int multirotor_position_control_params_init ();

	/* global variables */
	// empty

#endif
