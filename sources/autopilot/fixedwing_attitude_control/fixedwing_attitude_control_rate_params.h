/**
 * @file fixedwing_attitude_control_rate_params.h
 *
 * Parameters defined for the fixedwing position control
 *
 */


#ifndef __FIXEDWING_ATTITUDE_CONTROL_RATE_PARAMS_H
#define __FIXEDWING_ATTITUDE_CONTROL_RATE_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct fixedwing_attitude_control_rate_parameters {
		float rollrate_p;
		float rollrate_i;
		float rollrate_awu;
		float pitchrate_p;
		float pitchrate_i;
		float pitchrate_awu;
		float yawrate_p;
		float yawrate_i;
		float yawrate_awu;
		float pitch_thr_ff;
	} fixedwing_attitude_control_rate_parameters;			/**< local copies of interesting parameters */

	struct fixedwing_attitude_control_rate_parameter_handles {
		param_t rollrate_p;
		param_t rollrate_i;
		param_t rollrate_awu;
		param_t pitchrate_p;
		param_t pitchrate_i;
		param_t pitchrate_awu;
		param_t yawrate_p;
		param_t yawrate_i;
		param_t yawrate_awu;
		param_t pitch_thr_ff;
	} fixedwing_attitude_control_rate_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int fixedwing_attitude_control_rate_params_define ();
	int fixedwing_attitude_control_rate_params_update ();
	int fixedwing_attitude_control_rate_params_init ();

	/* global variables */
	// empty

#endif
