/**
 * @file fw_attitude_control_params.c
 *
 * Parameters defined for the attitude controller
 *
 */


#ifndef __FW_ATTITUDE_CONTROL_PARAMS_H
#define __FW_ATTITUDE_CONTROL_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct fw_attitude_control_parameters {
		float tconst;
		float p_p;
		float p_d;
		float p_i;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float p_roll_feedforward;
		float r_p;
		float r_d;
		float r_i;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_roll_feedforward;
		float y_integrator_max;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;
	} _fw_attitude_control_parameters;			/**< local copies of interesting parameters */

	struct fw_attitude_control_parameter_handles {
		param_t tconst;
		param_t p_p;
		param_t p_d;
		param_t p_i;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t p_roll_feedforward;
		param_t r_p;
		param_t r_d;
		param_t r_i;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_d;
		param_t y_roll_feedforward;
		param_t y_integrator_max;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;
	} _fw_attitude_control_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int fw_attitude_control_params_define ();
	int fw_attitude_control_parameters_update();
	int fw_attitude_control_param_init ();

	/* global variables */
	// empty

#endif
