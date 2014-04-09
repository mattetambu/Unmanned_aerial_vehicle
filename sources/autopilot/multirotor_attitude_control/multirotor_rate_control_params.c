/**
 * @file multirotor_rate_control_params.c
 *
 * Parameters defined for the multirotor rate control
 *
 */


#include "../../uav_library/param/param.h"

#include "multirotor_rate_control_params.h"


/*
 * Controller parameters
 *
 */
int multirotor_rate_control_params_define ()
{
	PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.3f); /* same on Flamewheel */
	PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.005f);
	PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.2f);
	//PARAM_DEFINE_FLOAT(MC_YAWRATE_AWU, 0.0f);
	//PARAM_DEFINE_FLOAT(MC_YAWRATE_LIM, 1.0f);

	PARAM_DEFINE_FLOAT(MC_ATTRATE_P, 0.09f); /* 0.15 F405 Flamewheel */
	PARAM_DEFINE_FLOAT(MC_ATTRATE_D, 0.002f);
	PARAM_DEFINE_FLOAT(MC_ATTRATE_I, 0.0f);
	//PARAM_DEFINE_FLOAT(MC_ATTRATE_AWU, 0.05f);
	//PARAM_DEFINE_FLOAT(MC_ATTRATE_LIM, 1.0f);	/**< roughly < 500 deg/s limit */

	return 0;
}

int multirotor_rate_control_params_update()
{
	PARAM_GET (multirotor_rate_control_parameter_handles.yawrate_p, &(multirotor_rate_control_parameters.yawrate_p));
	PARAM_GET (multirotor_rate_control_parameter_handles.yawrate_i, &(multirotor_rate_control_parameters.yawrate_i));
	PARAM_GET (multirotor_rate_control_parameter_handles.yawrate_d, &(multirotor_rate_control_parameters.yawrate_d));
	//PARAM_GET (multirotor_rate_control_parameter_handles.yawrate_awu, &(multirotor_rate_control_parameters.yawrate_awu));
	//PARAM_GET (multirotor_rate_control_parameter_handles.yawrate_lim, &(multirotor_rate_control_parameters.yawrate_lim));

	PARAM_GET (multirotor_rate_control_parameter_handles.attrate_p, &(multirotor_rate_control_parameters.attrate_p));
	PARAM_GET (multirotor_rate_control_parameter_handles.attrate_i, &(multirotor_rate_control_parameters.attrate_i));
	PARAM_GET (multirotor_rate_control_parameter_handles.attrate_d, &(multirotor_rate_control_parameters.attrate_d));
	//PARAM_GET (multirotor_rate_control_parameter_handles.attrate_awu, &(multirotor_rate_control_parameters.attrate_awu));
	//PARAM_GET (multirotor_rate_control_parameter_handles.attrate_lim, &(multirotor_rate_control_parameters.attrate_lim));

	return 0;
}


int multirotor_rate_control_params_init ()
{
	/* PID parameters */
	multirotor_rate_control_parameter_handles.yawrate_p 	=	param_find("MC_YAWRATE_P");
	multirotor_rate_control_parameter_handles.yawrate_i 	=	param_find("MC_YAWRATE_I");
	multirotor_rate_control_parameter_handles.yawrate_d 	=	param_find("MC_YAWRATE_D");
	//multirotor_rate_control_parameter_handles.yawrate_awu 	=	param_find("MC_YAWRATE_AWU");
	//multirotor_rate_control_parameter_handles.yawrate_lim 	=	param_find("MC_YAWRATE_LIM");

	multirotor_rate_control_parameter_handles.attrate_p 	= 	param_find("MC_ATTRATE_P");
	multirotor_rate_control_parameter_handles.attrate_i 	= 	param_find("MC_ATTRATE_I");
	multirotor_rate_control_parameter_handles.attrate_d 	= 	param_find("MC_ATTRATE_D");
	//multirotor_rate_control_parameter_handles.attrate_awu 	= 	param_find("MC_ATTRATE_AWU");
	//multirotor_rate_control_parameter_handles.attrate_lim 	= 	param_find("MC_ATTRATE_LIM");

	return multirotor_rate_control_params_update();
}
