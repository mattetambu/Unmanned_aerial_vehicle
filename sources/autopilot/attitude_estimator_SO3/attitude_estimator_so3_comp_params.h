/**
 * @file attitude_estimator_so3_comp_params.h
 *
 * Parameters defined for attitude estimator
 */


#ifndef __ATTITUDE_ESTIMATOR_SO3_COMP_PARAMS_H
#define __ATTITUDE_ESTIMATOR_SO3_COMP_PARAMS_H

	#include <stdint.h>
	#include "../../uav_library/param/param.h"

	struct attitude_estimator_so3_comp_params {
		float Kp;
		float Ki;
		float roll_off;
		float pitch_off;
		float yaw_off;
	} so3_comp_params;			/**< local copies of interesting parameters */

	struct attitude_estimator_so3_comp_param_handles {
		param_t Kp, Ki;
		param_t roll_off, pitch_off, yaw_off;
	} so3_comp_param_handles;	/**< handles for interesting parameters */


	/* function prototypes */
	int non_linear_SO3_AHRS_comp_params_define ();
	int non_linear_SO3_AHRS_comp_params_update ();
	int non_linear_SO3_AHRS_comp_params_init ();

	/* global variables */
	// empty

#endif
