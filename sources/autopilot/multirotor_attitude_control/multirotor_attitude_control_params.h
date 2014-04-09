/**
 * @file multirotor_attitude_control_params.h
 *
 * Parameters defined for the multirotor attitude control
 *
 */


#ifndef __MULTIROTOR_ATTITUDE_CONTROL_PARAMS_H
#define __MULTIROTOR_ATTITUDE_CONTROL_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct multirotor_attitude_control_parameters {
		float yaw_p;
		float yaw_i;
		float yaw_d;
		//float yaw_awu;
		//float yaw_lim;

		float att_p;
		float att_i;
		float att_d;
		//float att_awu;
		//float att_lim;

		//float att_xoff;
		//float att_yoff;
	} multirotor_attitude_control_parameters;			/**< local copies of interesting parameters */

	struct multirotor_attitude_control_parameter_handles {
		param_t yaw_p;
		param_t yaw_i;
		param_t yaw_d;
		//param_t yaw_awu;
		//param_t yaw_lim;

		param_t att_p;
		param_t att_i;
		param_t att_d;
		//param_t att_awu;
		//param_t att_lim;

		//param_t att_xoff;
		//param_t att_yoff;
	} multirotor_attitude_control_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int multirotor_attitude_control_params_define ();
	int multirotor_attitude_control_params_update ();
	int multirotor_attitude_control_params_init ();

	/* global variables */
	// empty

#endif
