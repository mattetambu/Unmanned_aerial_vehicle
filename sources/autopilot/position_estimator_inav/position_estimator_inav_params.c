/**
 * @file position_estimator_inav_params.c
 *
 * Parameters defined for the position estimator
 *
 */


#include "../../uav_library/param/param.h"

#include "position_estimator_inav_params.h"


/*
 * Position estimator parameters
 *
 */
int position_estimator_inav_params_define ()
{
	PARAM_DEFINE_FLOAT(INAV_W_Z_BARO, 0.5f);
	PARAM_DEFINE_FLOAT(INAV_W_Z_GPS_P, 0.005f);
	PARAM_DEFINE_FLOAT(INAV_W_Z_ACC, 20.0f);
	PARAM_DEFINE_FLOAT(INAV_W_Z_SONAR, 3.0f);
	PARAM_DEFINE_FLOAT(INAV_W_XY_GPS_P, 1.0f);
	PARAM_DEFINE_FLOAT(INAV_W_XY_GPS_V, 2.0f);
	PARAM_DEFINE_FLOAT(INAV_W_XY_ACC, 20.0f);
	PARAM_DEFINE_FLOAT(INAV_W_XY_FLOW, 5.0f);
	PARAM_DEFINE_FLOAT(INAV_W_GPS_FLOW, 0.1f);
	PARAM_DEFINE_FLOAT(INAV_W_ACC_BIAS, 0.05f);
	PARAM_DEFINE_FLOAT(INAV_FLOW_K, 0.0165f);
	PARAM_DEFINE_FLOAT(INAV_FLOW_Q_MIN, 0.5f);
	PARAM_DEFINE_FLOAT(INAV_SONAR_FILT, 0.05f);
	PARAM_DEFINE_FLOAT(INAV_SONAR_ERR, 0.5f);
	PARAM_DEFINE_FLOAT(INAV_LAND_T, 3.0f);
	PARAM_DEFINE_FLOAT(INAV_LAND_DISP, 0.7f);
	PARAM_DEFINE_FLOAT(INAV_LAND_THR, 0.3f);

	return 0;
}


int position_estimator_inav_params_update()
{
	PARAM_GET (position_estimator_inav_parameter_handles.w_z_baro, &(position_estimator_inav_parameters.w_z_baro));
	PARAM_GET (position_estimator_inav_parameter_handles.w_z_gps_p, &(position_estimator_inav_parameters.w_z_gps_p));
	PARAM_GET (position_estimator_inav_parameter_handles.w_z_acc, &(position_estimator_inav_parameters.w_z_acc));
	PARAM_GET (position_estimator_inav_parameter_handles.w_z_sonar, &(position_estimator_inav_parameters.w_z_sonar));
	PARAM_GET (position_estimator_inav_parameter_handles.w_xy_gps_p, &(position_estimator_inav_parameters.w_xy_gps_p));
	PARAM_GET (position_estimator_inav_parameter_handles.w_xy_gps_v, &(position_estimator_inav_parameters.w_xy_gps_v));
	PARAM_GET (position_estimator_inav_parameter_handles.w_xy_acc, &(position_estimator_inav_parameters.w_xy_acc));
	PARAM_GET (position_estimator_inav_parameter_handles.w_xy_flow, &(position_estimator_inav_parameters.w_xy_flow));
	PARAM_GET (position_estimator_inav_parameter_handles.w_gps_flow, &(position_estimator_inav_parameters.w_gps_flow));
	PARAM_GET (position_estimator_inav_parameter_handles.w_acc_bias, &(position_estimator_inav_parameters.w_acc_bias));
	PARAM_GET (position_estimator_inav_parameter_handles.flow_k, &(position_estimator_inav_parameters.flow_k));
	PARAM_GET (position_estimator_inav_parameter_handles.flow_q_min, &(position_estimator_inav_parameters.flow_q_min));
	PARAM_GET (position_estimator_inav_parameter_handles.sonar_filt, &(position_estimator_inav_parameters.sonar_filt));
	PARAM_GET (position_estimator_inav_parameter_handles.sonar_err, &(position_estimator_inav_parameters.sonar_err));
	PARAM_GET (position_estimator_inav_parameter_handles.land_t, &(position_estimator_inav_parameters.land_t));
	PARAM_GET (position_estimator_inav_parameter_handles.land_disp, &(position_estimator_inav_parameters.land_disp));
	PARAM_GET (position_estimator_inav_parameter_handles.land_thr, &(position_estimator_inav_parameters.land_thr));

	return 0;
}


int position_estimator_inav_params_init ()
{
	position_estimator_inav_parameter_handles.w_z_baro = param_find("INAV_W_Z_BARO");
	position_estimator_inav_parameter_handles.w_z_gps_p = param_find("INAV_W_Z_GPS_P");
	position_estimator_inav_parameter_handles.w_z_acc = param_find("INAV_W_Z_ACC");
	position_estimator_inav_parameter_handles.w_z_sonar = param_find("INAV_W_Z_SONAR");
	position_estimator_inav_parameter_handles.w_xy_gps_p = param_find("INAV_W_XY_GPS_P");
	position_estimator_inav_parameter_handles.w_xy_gps_v = param_find("INAV_W_XY_GPS_V");
	position_estimator_inav_parameter_handles.w_xy_acc = param_find("INAV_W_XY_ACC");
	position_estimator_inav_parameter_handles.w_xy_flow = param_find("INAV_W_XY_FLOW");
	position_estimator_inav_parameter_handles.w_gps_flow = param_find("INAV_W_GPS_FLOW");
	position_estimator_inav_parameter_handles.w_acc_bias = param_find("INAV_W_ACC_BIAS");
	position_estimator_inav_parameter_handles.flow_k = param_find("INAV_FLOW_K");
	position_estimator_inav_parameter_handles.flow_q_min = param_find("INAV_FLOW_Q_MIN");
	position_estimator_inav_parameter_handles.sonar_filt = param_find("INAV_SONAR_FILT");
	position_estimator_inav_parameter_handles.sonar_err = param_find("INAV_SONAR_ERR");
	position_estimator_inav_parameter_handles.land_t = param_find("INAV_LAND_T");
	position_estimator_inav_parameter_handles.land_disp = param_find("INAV_LAND_DISP");
	position_estimator_inav_parameter_handles.land_thr = param_find("INAV_LAND_THR");

	return position_estimator_inav_params_update();
}
