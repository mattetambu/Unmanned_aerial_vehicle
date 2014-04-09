/**
 * @file position_estimator_inav_params.h
 *
 * Parameters defined for the position estimator
 *
 */


#ifndef __POSITION_ESTIMATOR_INAV_PARAMS_H
#define __POSITION_ESTIMATOR_INAV_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct position_estimator_inav_parameters {
		float w_z_baro;
		float w_z_gps_p;
		float w_z_acc;
		float w_z_sonar;
		float w_xy_gps_p;
		float w_xy_gps_v;
		float w_xy_acc;
		float w_xy_flow;
		float w_gps_flow;
		float w_acc_bias;
		float flow_k;
		float flow_q_min;
		float sonar_filt;
		float sonar_err;
		float land_t;
		float land_disp;
		float land_thr;
	} position_estimator_inav_parameters;			/**< local copies of interesting parameters */

	struct position_estimator_inav_parameter_handles {
		param_t w_z_baro;
		param_t w_z_gps_p;
		param_t w_z_acc;
		param_t w_z_sonar;
		param_t w_xy_gps_p;
		param_t w_xy_gps_v;
		param_t w_xy_acc;
		param_t w_xy_flow;
		param_t w_gps_flow;
		param_t w_acc_bias;
		param_t flow_k;
		param_t flow_q_min;
		param_t sonar_filt;
		param_t sonar_err;
		param_t land_t;
		param_t land_disp;
		param_t land_thr;
	} position_estimator_inav_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int position_estimator_inav_params_define ();
	int position_estimator_inav_params_update ();
	int position_estimator_inav_params_init ();

	/* global variables */
	// empty

#endif
