/**
 * @file position_estimator_mc_params.h
 *
 * Parameters defined for the position estimator
 *
 */


#ifndef __POSITION_ESTIMATOR_MC_PARAMS_H
#define __POSITION_ESTIMATOR_MC_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct position_estimator_mc_parameters {
	float addNoise;
	float sigma;
	float R;
	int baro; /* consider barometer */
	} position_estimator_mc_parameters;			/**< local copies of interesting parameters */

	struct position_estimator_mc_parameter_handles {
		param_t addNoise;
		param_t sigma;
		param_t r;
		param_t baro_param_handle;
	} position_estimator_mc_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int position_estimator_mc_params_define ();
	int position_estimator_mc_params_update ();
	int position_estimator_mc_params_init ();

	/* global variables */
	// empty

#endif
