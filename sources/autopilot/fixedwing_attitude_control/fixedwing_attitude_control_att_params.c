/**
 * @file fixedwing_attitude_control_att_params.c
 *
 * Parameters defined for the fixedwing attitude control
 *
 */


#include "../../uav_library/param/param.h"
#include "../../uav_library/math/limits.h"

#include "fixedwing_attitude_control_att_params.h"


/*
 * Controller parameters
 *
 */
int fixedwing_attitude_control_att_params_define ()
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

int fixedwing_attitude_control_att_params_update()
{
	PARAM_GET (fixedwing_attitude_control_att_parameter_handles.roll_p, &(fixedwing_attitude_control_att_parameters.roll_p));
	PARAM_GET (fixedwing_attitude_control_att_parameter_handles.rollrate_lim, &(fixedwing_attitude_control_att_parameters.rollrate_lim));
	PARAM_GET (fixedwing_attitude_control_att_parameter_handles.pitch_p, &(fixedwing_attitude_control_att_parameters.pitch_p));
	PARAM_GET (fixedwing_attitude_control_att_parameter_handles.pitchrate_lim, &(fixedwing_attitude_control_att_parameters.pitchrate_lim));
	PARAM_GET (fixedwing_attitude_control_att_parameter_handles.yawrate_lim, &(fixedwing_attitude_control_att_parameters.yawrate_lim));
	PARAM_GET (fixedwing_attitude_control_att_parameter_handles.pitch_roll_compensation_p, &(fixedwing_attitude_control_att_parameters.pitch_roll_compensation_p));

	return 0;
}


int fixedwing_attitude_control_att_params_init ()
{
	/* PID parameters */
	fixedwing_attitude_control_att_parameter_handles.roll_p 		=	param_find("FW_ROLL_P");
	fixedwing_attitude_control_att_parameter_handles.rollrate_lim =	param_find("FW_ROLLR_LIM");
	fixedwing_attitude_control_att_parameter_handles.pitch_p 		=	param_find("FW_PITCH_P");
	fixedwing_attitude_control_att_parameter_handles.pitchrate_lim =	param_find("FW_PITCHR_LIM");
	fixedwing_attitude_control_att_parameter_handles.yawrate_lim =	param_find("FW_YAWR_LIM");
	fixedwing_attitude_control_att_parameter_handles.pitch_roll_compensation_p = param_find("FW_PITCH_RCOMP");

	return fixedwing_attitude_control_att_params_update();
}
