/**
 * @file multirotor_attitude_control_params.c
 *
 * Parameters defined for the multirotor attitude control
 *
 */


#include "../../uav_library/param/param.h"

#include "multirotor_attitude_control_params.h"


/*
 * Controller parameters
 *
 */
int multirotor_attitude_control_params_define ()
{
	PARAM_DEFINE_FLOAT(MC_YAWPOS_P, 2.0f);
	PARAM_DEFINE_FLOAT(MC_YAWPOS_I, 0.15f);
	PARAM_DEFINE_FLOAT(MC_YAWPOS_D, 0.0f);
	//PARAM_DEFINE_FLOAT(MC_YAWPOS_AWU, 1.0f);
	//PARAM_DEFINE_FLOAT(MC_YAWPOS_LIM, 3.0f);

	PARAM_DEFINE_FLOAT(MC_ATT_P, 6.8f);
	PARAM_DEFINE_FLOAT(MC_ATT_I, 0.0f);
	PARAM_DEFINE_FLOAT(MC_ATT_D, 0.0f);
	//PARAM_DEFINE_FLOAT(MC_ATT_AWU, 0.05f);
	//PARAM_DEFINE_FLOAT(MC_ATT_LIM, 0.4f);

	//PARAM_DEFINE_FLOAT(MC_ATT_XOFF, 0.0f);
	//PARAM_DEFINE_FLOAT(MC_ATT_YOFF, 0.0f);

	return 0;
}

int multirotor_attitude_control_params_update()
{
	PARAM_GET (multirotor_attitude_control_parameter_handles.yaw_p, &(multirotor_attitude_control_parameters.yaw_p));
	PARAM_GET (multirotor_attitude_control_parameter_handles.yaw_i, &(multirotor_attitude_control_parameters.yaw_i));
	PARAM_GET (multirotor_attitude_control_parameter_handles.yaw_d, &(multirotor_attitude_control_parameters.yaw_d));
	//PARAM_GET (multirotor_attitude_control_parameter_handles.yaw_awu, &(multirotor_attitude_control_parameters.yaw_awu));
	//PARAM_GET (multirotor_attitude_control_parameter_handles.yaw_lim, &(multirotor_attitude_control_parameters.yaw_lim));

	PARAM_GET (multirotor_attitude_control_parameter_handles.att_p, &(multirotor_attitude_control_parameters.att_p));
	PARAM_GET (multirotor_attitude_control_parameter_handles.att_i, &(multirotor_attitude_control_parameters.att_i));
	PARAM_GET (multirotor_attitude_control_parameter_handles.att_d, &(multirotor_attitude_control_parameters.att_d));
	//PARAM_GET (multirotor_attitude_control_parameter_handles.att_awu, &(multirotor_attitude_control_parameters.att_awu));
	//PARAM_GET (multirotor_attitude_control_parameter_handles.att_lim, &(multirotor_attitude_control_parameters.att_lim));

	//PARAM_GET (multirotor_attitude_control_parameter_handles.att_xoff, &(multirotor_attitude_control_parameters.att_xoff));
	//PARAM_GET (multirotor_attitude_control_parameter_handles.att_yoff, &(multirotor_attitude_control_parameters.att_yoff));

	return 0;
}


int multirotor_attitude_control_params_init ()
{
	/* PID parameters */
	multirotor_attitude_control_parameter_handles.yaw_p 	=	param_find("MC_YAWPOS_P");
	multirotor_attitude_control_parameter_handles.yaw_i 	=	param_find("MC_YAWPOS_I");
	multirotor_attitude_control_parameter_handles.yaw_d 	=	param_find("MC_YAWPOS_D");
	//multirotor_attitude_control_parameter_handles.yaw_awu 	=	param_find("MC_YAWPOS_AWU");
	//multirotor_attitude_control_parameter_handles.yaw_lim 	=	param_find("MC_YAWPOS_LIM");

	multirotor_attitude_control_parameter_handles.att_p 	= 	param_find("MC_ATT_P");
	multirotor_attitude_control_parameter_handles.att_i 	= 	param_find("MC_ATT_I");
	multirotor_attitude_control_parameter_handles.att_d 	= 	param_find("MC_ATT_D");
	//multirotor_attitude_control_parameter_handles.att_awu 	= 	param_find("MC_ATT_AWU");
	//multirotor_attitude_control_parameter_handles.att_lim 	= 	param_find("MC_ATT_LIM");

	//multirotor_attitude_control_parameter_handles.att_xoff 	= 	param_find("MC_ATT_XOFF");
	//multirotor_attitude_control_parameter_handles.att_yoff 	= 	param_find("MC_ATT_YOFF");

	return multirotor_attitude_control_params_update();
}
