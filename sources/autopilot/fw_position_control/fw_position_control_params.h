/**
 * @file fw_position_control_params.h
 *
 * Parameters defined for the L1 position control
 *
 */


#ifndef __FW_POSITION_CONTROL_PARAMS_H
#define __FW_POSITION_CONTROL_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct fw_position_control_parameters {
		float l1_period;
		float l1_damping;

		float time_const;
		float min_sink_rate;
		float max_sink_rate;
		float max_climb_rate;
		float throttle_damp;
		float integrator_gain;
		float vertical_accel_limit;
		float height_comp_filter_omega;
		float speed_comp_filter_omega;
		float roll_throttle_compensation;
		float speed_weight;
		float pitch_damping;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_cruise;

		float throttle_land_max;

		float loiter_hold_radius;
	} _fw_position_control_parameters;			/**< local copies of interesting parameters */

	struct fw_position_control_parameter_handles {
		param_t l1_period;
		param_t l1_damping;

		param_t time_const;
		param_t min_sink_rate;
		param_t max_sink_rate;
		param_t max_climb_rate;
		param_t throttle_damp;
		param_t integrator_gain;
		param_t vertical_accel_limit;
		param_t height_comp_filter_omega;
		param_t speed_comp_filter_omega;
		param_t roll_throttle_compensation;
		param_t speed_weight;
		param_t pitch_damping;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;

		param_t throttle_land_max;

		param_t loiter_hold_radius;
	} _fw_position_control_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int fw_position_control_params_define ();
	int fw_position_control_params_update ();
	int fw_position_control_params_init ();

	/* global variables */
	// empty

#endif
