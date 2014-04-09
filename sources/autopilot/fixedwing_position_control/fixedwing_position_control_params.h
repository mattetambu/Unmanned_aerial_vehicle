/**
 * @file fixedwing_position_control_params.h
 *
 * Parameters defined for the fixedwing position control
 *
 */


#ifndef __FIXEDWING_POSITION_CONTROL_PARAMS_H
#define __FIXEDWING_POSITION_CONTROL_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct fixedwing_position_control_parameters {
		float heading_p;
		float headingr_p;
		float headingr_i;
		float headingr_lim;
		float xtrack_p;
		float altitude_p;
		float roll_lim;
		float pitch_lim;
	} fixedwing_position_control_parameters;			/**< local copies of interesting parameters */

	struct fixedwing_position_control_parameter_handles {
		param_t heading_p;
		param_t headingr_p;
		param_t headingr_i;
		param_t headingr_lim;
		param_t xtrack_p;
		param_t altitude_p;
		param_t roll_lim;
		param_t pitch_lim;
	} fixedwing_position_control_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int fixedwing_position_control_params_define ();
	int fixedwing_position_control_params_update ();
	int fixedwing_position_control_params_init ();

	/* global variables */
	// empty

#endif
