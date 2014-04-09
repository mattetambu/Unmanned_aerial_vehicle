/**
 * @file position_estimator_mc_params.c
 *
 * Parameters defined for the position estimator
 *
 */


#include "../../uav_library/param/param.h"

#include "position_estimator_mc_params.h"


/*
 * Position estimator parameters
 *
 */
int position_estimator_mc_params_define ()
{
	/* Kalman Filter covariances */
	/* gps measurement noise standard deviation */
	PARAM_DEFINE_FLOAT(POS_EST_ADDN, 1.0f);
	PARAM_DEFINE_FLOAT(POS_EST_SIGMA, 0.0f);
	PARAM_DEFINE_FLOAT(POS_EST_R, 1.0f);
	PARAM_DEFINE_INT(POS_EST_BARO, 0.0f);

	return 0;
}

int position_estimator_mc_params_update()
{
	PARAM_GET (position_estimator_mc_parameter_handles.addNoise, &(position_estimator_mc_parameters.addNoise));
	PARAM_GET (position_estimator_mc_parameter_handles.sigma, &(position_estimator_mc_parameters.sigma));
	PARAM_GET (position_estimator_mc_parameter_handles.r, &(position_estimator_mc_parameters.R));
	PARAM_GET (position_estimator_mc_parameter_handles.baro_param_handle, &(position_estimator_mc_parameters.baro));

	return 0;
}


int position_estimator_mc_params_init ()
{
	position_estimator_mc_parameter_handles.addNoise	= param_find("POS_EST_ADDN");
	position_estimator_mc_parameter_handles.sigma = param_find("POS_EST_SIGMA");
	position_estimator_mc_parameter_handles.r = param_find("POS_EST_R");
	position_estimator_mc_parameter_handles.baro_param_handle = param_find("POS_EST_BARO");

	return position_estimator_mc_params_update();
}
