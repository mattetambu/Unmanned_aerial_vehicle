/**
 * @file fixedwing_position_control_params.c
 *
 * Parameters defined for the fixedwing position control
 *
 */


#include "../../uav_library/param/param.h"
#include "../../uav_library/math/limits.h"

#include "fixedwing_position_control_params.h"


/*
 * Controller parameters
 *
 */
int fixedwing_position_control_params_define ()
{
	PARAM_DEFINE_FLOAT(FW_HEAD_P, 0.1f);
	PARAM_DEFINE_FLOAT(FW_HEADR_I, 0.1f);
	PARAM_DEFINE_FLOAT(FW_HEADR_LIM, 1.5f);		//TODO: think about reasonable value
	PARAM_DEFINE_FLOAT(FW_XTRACK_P, 0.01745f);	// Radians per meter off track
	PARAM_DEFINE_FLOAT(FW_ALT_P, 0.1f);
	PARAM_DEFINE_FLOAT(FW_ROLL_LIM, 0.7f);		// Roll angle limit in radians
	PARAM_DEFINE_FLOAT(FW_HEADR_P, 0.1f);
	PARAM_DEFINE_FLOAT(FW_PITCH_LIM, 0.35f);	// Pitch angle limit in radians per second

	return 0;
}

int fixedwing_position_control_params_update()
{
	PARAM_GET(fixedwing_position_control_parameter_handles.heading_p, &(fixedwing_position_control_parameters.heading_p));
	PARAM_GET(fixedwing_position_control_parameter_handles.headingr_p, &(fixedwing_position_control_parameters.headingr_p));
	PARAM_GET(fixedwing_position_control_parameter_handles.headingr_i, &(fixedwing_position_control_parameters.headingr_i));
	PARAM_GET(fixedwing_position_control_parameter_handles.headingr_lim, &(fixedwing_position_control_parameters.headingr_lim));
	PARAM_GET(fixedwing_position_control_parameter_handles.xtrack_p, &(fixedwing_position_control_parameters.xtrack_p));
	PARAM_GET(fixedwing_position_control_parameter_handles.altitude_p, &(fixedwing_position_control_parameters.altitude_p));
	PARAM_GET(fixedwing_position_control_parameter_handles.roll_lim, &(fixedwing_position_control_parameters.roll_lim));
	PARAM_GET(fixedwing_position_control_parameter_handles.pitch_lim, &(fixedwing_position_control_parameters.pitch_lim));

	return 0;
}


int fixedwing_position_control_params_init ()
{
	/* PID parameters */
	fixedwing_position_control_parameter_handles.heading_p 		=	param_find("FW_HEAD_P");
	fixedwing_position_control_parameter_handles.headingr_p 		=	param_find("FW_HEADR_P");
	fixedwing_position_control_parameter_handles.headingr_i 		=	param_find("FW_HEADR_I");
	fixedwing_position_control_parameter_handles.headingr_lim 	=	param_find("FW_HEADR_LIM");
	fixedwing_position_control_parameter_handles.xtrack_p 		=	param_find("FW_XTRACK_P");
	fixedwing_position_control_parameter_handles.altitude_p 		=	param_find("FW_ALT_P");
	fixedwing_position_control_parameter_handles.roll_lim 		=	param_find("FW_ROLL_LIM");
	fixedwing_position_control_parameter_handles.pitch_lim 		=	param_find("FW_PITCH_LIM");

	return fixedwing_position_control_params_update();
}
