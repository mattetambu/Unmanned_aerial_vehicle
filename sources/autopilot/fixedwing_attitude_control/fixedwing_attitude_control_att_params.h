/**
 * @file fixedwing_attitude_control_att_params.h
 *
 * Parameters defined for the fixedwing attitude control
 *
 */


#ifndef __FIXEDWING_ATTITUDE_CONTROL_ATT_PARAMS_H
#define __FIXEDWING_ATTITUDE_CONTROL_ATT_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct fixedwing_attitude_control_att_parameters {
		float roll_p;
		float rollrate_lim;
		float pitch_p;
		float pitchrate_lim;
		float yawrate_lim;
		float pitch_roll_compensation_p;
	} fixedwing_attitude_control_att_parameters;			/**< local copies of interesting parameters */

	struct fixedwing_attitude_control_att_parameter_handles {
		param_t roll_p;
		param_t rollrate_lim;
		param_t pitch_p;
		param_t pitchrate_lim;
		param_t yawrate_lim;
		param_t pitch_roll_compensation_p;
	} fixedwing_attitude_control_att_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int fixedwing_attitude_control_att_params_define ();
	int fixedwing_attitude_control_att_params_update ();
	int fixedwing_attitude_control_att_params_init ();

	/* global variables */
	// empty

#endif
