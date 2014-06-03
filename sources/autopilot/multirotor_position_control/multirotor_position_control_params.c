/**
 * @file multirotor_position_control_params.c
 *
 * Parameters defined for the multirotor position control
 *
 */


#include "../../uav_library/param/param.h"

#include "multirotor_position_control_params.h"


/*
 * Controller parameters
 *
 */
int multirotor_position_control_params_define ()
{
	/* controller parameters */
	PARAM_DEFINE_FLOAT(NAV_TAKEOFF_ALT, 15.0f);
	PARAM_DEFINE_FLOAT(NAV_TAKEOFF_GAP, 0.5f);
	PARAM_DEFINE_FLOAT(MPC_THR_MIN, 0.2f);
	PARAM_DEFINE_FLOAT(MPC_THR_MAX, 0.8f);
	PARAM_DEFINE_FLOAT(MPC_Z_P, 1.0f);
	PARAM_DEFINE_FLOAT(MPC_Z_D, 0.0f);
	PARAM_DEFINE_FLOAT(MPC_Z_VEL_P, 0.1f);
	PARAM_DEFINE_FLOAT(MPC_Z_VEL_I, 0.0f);
	PARAM_DEFINE_FLOAT(MPC_Z_VEL_D, 0.0f);
	PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX, 3.0f);
	PARAM_DEFINE_FLOAT(MPC_XY_P, 0.2f);
	PARAM_DEFINE_FLOAT(MPC_XY_D, 0.0f);
	PARAM_DEFINE_FLOAT(MPC_XY_VEL_P, 0.2f);
	PARAM_DEFINE_FLOAT(MPC_XY_VEL_I, 0.0f);
	PARAM_DEFINE_FLOAT(MPC_XY_VEL_D, 0.0f);
	PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 5.0f);
	PARAM_DEFINE_FLOAT(MPC_TILT_MAX, 0.5f);

	return 0;
}

int multirotor_position_control_params_update()
{
	PARAM_GET (multirotor_position_control_parameter_handles.takeoff_alt, &(multirotor_position_control_parameters.takeoff_alt));
	PARAM_GET (multirotor_position_control_parameter_handles.takeoff_gap, &(multirotor_position_control_parameters.takeoff_gap));
	PARAM_GET (multirotor_position_control_parameter_handles.thr_min, &(multirotor_position_control_parameters.thr_min));
	PARAM_GET (multirotor_position_control_parameter_handles.thr_max, &(multirotor_position_control_parameters.thr_max));
	PARAM_GET (multirotor_position_control_parameter_handles.z_p, &(multirotor_position_control_parameters.z_p));
	PARAM_GET (multirotor_position_control_parameter_handles.z_d, &(multirotor_position_control_parameters.z_d));
	PARAM_GET (multirotor_position_control_parameter_handles.z_vel_p, &(multirotor_position_control_parameters.z_vel_p));
	PARAM_GET (multirotor_position_control_parameter_handles.z_vel_i, &(multirotor_position_control_parameters.z_vel_i));
	PARAM_GET (multirotor_position_control_parameter_handles.z_vel_d, &(multirotor_position_control_parameters.z_vel_d));
	PARAM_GET (multirotor_position_control_parameter_handles.z_vel_max, &(multirotor_position_control_parameters.z_vel_max));
	PARAM_GET (multirotor_position_control_parameter_handles.xy_p, &(multirotor_position_control_parameters.xy_p));
	PARAM_GET (multirotor_position_control_parameter_handles.xy_d, &(multirotor_position_control_parameters.xy_d));
	PARAM_GET (multirotor_position_control_parameter_handles.xy_vel_p, &(multirotor_position_control_parameters.xy_vel_p));
	PARAM_GET (multirotor_position_control_parameter_handles.xy_vel_i, &(multirotor_position_control_parameters.xy_vel_i));
	PARAM_GET (multirotor_position_control_parameter_handles.xy_vel_d, &(multirotor_position_control_parameters.xy_vel_d));
	PARAM_GET (multirotor_position_control_parameter_handles.xy_vel_max, &(multirotor_position_control_parameters.xy_vel_max));
	PARAM_GET (multirotor_position_control_parameter_handles.tilt_max, &(multirotor_position_control_parameters.tilt_max));

	PARAM_GET (multirotor_position_control_parameter_handles.rc_scale_pitch, &(multirotor_position_control_parameters.rc_scale_pitch));
	PARAM_GET (multirotor_position_control_parameter_handles.rc_scale_roll, &(multirotor_position_control_parameters.rc_scale_roll));
	PARAM_GET (multirotor_position_control_parameter_handles.rc_scale_yaw, &(multirotor_position_control_parameters.rc_scale_yaw));

	return 0;
}


int multirotor_position_control_params_init ()
{
	multirotor_position_control_parameter_handles.takeoff_alt = param_find("NAV_TAKEOFF_ALT");
	multirotor_position_control_parameter_handles.takeoff_gap = param_find("NAV_TAKEOFF_GAP");
	multirotor_position_control_parameter_handles.thr_min 	=	param_find("MPC_THR_MIN");
	multirotor_position_control_parameter_handles.thr_max 	=	param_find("MPC_THR_MAX");
	multirotor_position_control_parameter_handles.z_p 	=	param_find("MPC_Z_P");
	multirotor_position_control_parameter_handles.z_d 	=	param_find("MPC_Z_D");
	multirotor_position_control_parameter_handles.z_vel_p 	=	param_find("MPC_Z_VEL_P");
	multirotor_position_control_parameter_handles.z_vel_i 	=	param_find("MPC_Z_VEL_I");
	multirotor_position_control_parameter_handles.z_vel_d 	=	param_find("MPC_Z_VEL_D");
	multirotor_position_control_parameter_handles.z_vel_max 	=	param_find("MPC_Z_VEL_MAX");
	multirotor_position_control_parameter_handles.xy_p 	=	param_find("MPC_XY_P");
	multirotor_position_control_parameter_handles.xy_d 	=	param_find("MPC_XY_D");
	multirotor_position_control_parameter_handles.xy_vel_p 	=	param_find("MPC_XY_VEL_P");
	multirotor_position_control_parameter_handles.xy_vel_i 	=	param_find("MPC_XY_VEL_I");
	multirotor_position_control_parameter_handles.xy_vel_d 	=	param_find("MPC_XY_VEL_D");
	multirotor_position_control_parameter_handles.xy_vel_max 	=	param_find("MPC_XY_VEL_MAX");
	multirotor_position_control_parameter_handles.tilt_max 	=	param_find("MPC_TILT_MAX");

	multirotor_position_control_parameter_handles.rc_scale_pitch    =   param_find("RC_SCALE_PITCH");
	multirotor_position_control_parameter_handles.rc_scale_roll    =   param_find("RC_SCALE_ROLL");
	multirotor_position_control_parameter_handles.rc_scale_yaw      =   param_find("RC_SCALE_YAW");

	return multirotor_position_control_params_update();
}
