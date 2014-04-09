/**
 * @file multirotor_rate_control_params.h
 *
 * Parameters defined for the multirotor rate control
 *
 */


#ifndef __MULTIROTOR_RATE_CONTROL_PARAMS_H
#define __MULTIROTOR_RATE_CONTROL_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct multirotor_rate_control_parameters {
		float yawrate_p;
		float yawrate_d;
		float yawrate_i;
		//float yawrate_awu;
		//float yawrate_lim;

		float attrate_p;
		float attrate_d;
		float attrate_i;
		//float attrate_awu;
		//float attrate_lim;

		float rate_lim;
	} multirotor_rate_control_parameters;			/**< local copies of interesting parameters */

	struct multirotor_rate_control_parameter_handles {
		param_t yawrate_p;
		param_t yawrate_i;
		param_t yawrate_d;
		//param_t yawrate_awu;
		//param_t yawrate_lim;

		param_t attrate_p;
		param_t attrate_i;
		param_t attrate_d;
		//param_t attrate_awu;
		//param_t attrate_lim;
	} multirotor_rate_control_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int multirotor_rate_control_params_define ();
	int multirotor_rate_control_params_update ();
	int multirotor_rate_control_params_init ();

	/* global variables */
	// empty

#endif
