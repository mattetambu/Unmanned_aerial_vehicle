/**
 * @file mr_attitude_control_params.h
 *
 * Parameters defined for the multirotor attitude control
 *
 */


#ifndef __MR_ATTITUDE_CONTROL_PARAMS_H
#define __MR_ATTITUDE_CONTROL_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"
	#include "../../uav_library/math/Vector3f.h"

	struct mr_attitude_control_parameters {
		Vector3f att_p;					/**< P gain for angular error */
		Vector3f rate_p;				/**< P gain for angular rate error */
		Vector3f rate_i;				/**< I gain for angular rate error */
		Vector3f rate_d;				/**< D gain for angular rate error */

		float yaw_ff;						/**< yaw control feed-forward */
		float rc_scale_yaw;
	} mr_attitude_control_parameters;			/**< local copies of interesting parameters */

	struct mr_attitude_control_parameter_handles {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;

		param_t yaw_ff;
		param_t rc_scale_yaw;
	} mr_attitude_control_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int mr_attitude_control_params_define ();
	int mr_attitude_control_params_update ();
	int mr_attitude_control_params_init ();

	/* global variables */
	// empty

#endif
