/**
 * @file fixedwing_attitude_control_rate_params.c
 *
 * Parameters defined for the fixedwing position control
 *
 */


#include "../../uav_library/param/param.h"
#include "../../uav_library/math/limits.h"

#include "fixedwing_attitude_control_rate_params.h"


/*
 * Controller parameters
 *
 */
int fixedwing_attitude_control_rate_params_define ()
{
	// Roll control parameters
	PARAM_DEFINE_FLOAT(FW_ROLLR_P, 0.9f);
	PARAM_DEFINE_FLOAT(FW_ROLLR_I, 0.2f);
	PARAM_DEFINE_FLOAT(FW_ROLLR_AWU, 0.9f);
	PARAM_DEFINE_FLOAT(FW_ROLLR_LIM, 0.7f);   // Roll rate limit in radians/sec, applies to the roll controller
	PARAM_DEFINE_FLOAT(FW_ROLL_P, 4.0f);
	PARAM_DEFINE_FLOAT(FW_PITCH_RCOMP, 0.1f);

	//Pitch control parameters
	PARAM_DEFINE_FLOAT(FW_PITCHR_P, 0.8f);
	PARAM_DEFINE_FLOAT(FW_PITCHR_I, 0.2f);
	PARAM_DEFINE_FLOAT(FW_PITCHR_AWU, 0.8f);
	PARAM_DEFINE_FLOAT(FW_PITCHR_LIM, 0.35f);   // Pitch rate limit in radians/sec, applies to the pitch controller
	PARAM_DEFINE_FLOAT(FW_PITCH_P, 8.0f);

	//Yaw control parameters					//XXX TODO this is copy paste, asign correct values
	PARAM_DEFINE_FLOAT(FW_YAWR_P, 0.3f);
	PARAM_DEFINE_FLOAT(FW_YAWR_I, 0.0f);
	PARAM_DEFINE_FLOAT(FW_YAWR_AWU, 0.0f);
	PARAM_DEFINE_FLOAT(FW_YAWR_LIM, 0.35f);   // Yaw rate limit in radians/sec

	/* feedforward compensation */
	PARAM_DEFINE_FLOAT(FW_PITCH_THR_P, 0.1f);	/**< throttle to pitch coupling feedforward */

	return 0;
}

int fixedwing_attitude_control_rate_params_update()
{
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.rollrate_p, &(fixedwing_attitude_control_rate_parameters.rollrate_p));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.rollrate_i, &(fixedwing_attitude_control_rate_parameters.rollrate_i));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.rollrate_awu, &(fixedwing_attitude_control_rate_parameters.rollrate_awu));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.pitchrate_p, &(fixedwing_attitude_control_rate_parameters.pitchrate_p));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.pitchrate_i, &(fixedwing_attitude_control_rate_parameters.pitchrate_i));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.pitchrate_awu, &(fixedwing_attitude_control_rate_parameters.pitchrate_awu));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.yawrate_p, &(fixedwing_attitude_control_rate_parameters.yawrate_p));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.yawrate_i, &(fixedwing_attitude_control_rate_parameters.yawrate_i));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.yawrate_awu, &(fixedwing_attitude_control_rate_parameters.yawrate_awu));
	PARAM_GET (fixedwing_attitude_control_rate_parameter_handles.pitch_thr_ff, &(fixedwing_attitude_control_rate_parameters.pitch_thr_ff));

	return 0;
}


int fixedwing_attitude_control_rate_params_init ()
{
	/* PID parameters */
	fixedwing_attitude_control_rate_parameter_handles.rollrate_p 	 =	param_find("FW_ROLLR_P");   //TODO define rate params for fixed wing
	fixedwing_attitude_control_rate_parameter_handles.rollrate_i 	 =	param_find("FW_ROLLR_I");
	fixedwing_attitude_control_rate_parameter_handles.rollrate_awu  =	param_find("FW_ROLLR_AWU");

	fixedwing_attitude_control_rate_parameter_handles.pitchrate_p 	 =	param_find("FW_PITCHR_P");
	fixedwing_attitude_control_rate_parameter_handles.pitchrate_i 	 =	param_find("FW_PITCHR_I");
	fixedwing_attitude_control_rate_parameter_handles.pitchrate_awu =	param_find("FW_PITCHR_AWU");

	fixedwing_attitude_control_rate_parameter_handles.yawrate_p 	 =	param_find("FW_YAWR_P");
	fixedwing_attitude_control_rate_parameter_handles.yawrate_i 	 =	param_find("FW_YAWR_I");
	fixedwing_attitude_control_rate_parameter_handles.yawrate_awu   =	param_find("FW_YAWR_AWU");
	fixedwing_attitude_control_rate_parameter_handles.pitch_thr_ff  =	param_find("FW_PITCH_THR_P");

	return fixedwing_attitude_control_rate_params_update();
}
