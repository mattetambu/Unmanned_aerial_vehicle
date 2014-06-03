/**
 * @file mr_attitude_control_params.c
 *
 * Parameters defined for the mr attitude control
 *
 */


#include "../../uav_library/param/param.h"
#include "mr_attitude_control_params.h"


/*
 * Controller parameters
 *
 */
int mr_attitude_control_params_define ()
{
	/**
	 * Roll P gain
	 *
	 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_ROLL_P, 6.0f);

	/**
	 * Roll rate P gain
	 *
	 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.1f);

	/**
	 * Roll rate I gain
	 *
	 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.0f);

	/**
	 * Roll rate D gain
	 *
	 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.002f);

	/**
	 * Pitch P gain
	 *
	 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
	 *
	 * @unit 1/s
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.0f);

	/**
	 * Pitch rate P gain
	 *
	 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.1f);

	/**
	 * Pitch rate I gain
	 *
	 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.0f);

	/**
	 * Pitch rate D gain
	 *
	 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.002f);

	/**
	 * Yaw P gain
	 *
	 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
	 *
	 * @unit 1/s
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_YAW_P, 2.0f);

	/**
	 * Yaw rate P gain
	 *
	 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.3f);

	/**
	 * Yaw rate I gain
	 *
	 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.0f);

	/**
	 * Yaw rate D gain
	 *
	 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
	 *
	 * @min 0.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

	/**
	 * Yaw feed forward
	 *
	 * Feed forward weight for manual yaw control. 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
	 *
	 * @min 0.0
	 * @max 1.0
	 * @group Multicopter Attitude Control
	 */
	PARAM_DEFINE_FLOAT(MC_YAW_FF, 0.5f);

	/**
	 * Yaw scaling factor
	 *
	 * @group Radio Calibration
	 */
	PARAM_DEFINE_FLOAT(RC_SCALE_YAW, 2.0f);

	return 0;
}

int mr_attitude_control_params_update()
{
	float v;

	/* roll */
	PARAM_GET(mr_attitude_control_parameter_handles.roll_p, &v);
	mr_attitude_control_parameters.att_p.data[0] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.roll_rate_p, &v);
	mr_attitude_control_parameters.rate_p.data[0] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.roll_rate_i, &v);
	mr_attitude_control_parameters.rate_i.data[0] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.roll_rate_d, &v);
	mr_attitude_control_parameters.rate_d.data[0] = v;

	/* pitch */
	PARAM_GET(mr_attitude_control_parameter_handles.pitch_p, &v);
	mr_attitude_control_parameters.att_p.data[1] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.pitch_rate_p, &v);
	mr_attitude_control_parameters.rate_p.data[1] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.pitch_rate_i, &v);
	mr_attitude_control_parameters.rate_i.data[1] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.pitch_rate_d, &v);
	mr_attitude_control_parameters.rate_d.data[1] = v;

	/* yaw */
	PARAM_GET(mr_attitude_control_parameter_handles.yaw_p, &v);
	mr_attitude_control_parameters.att_p.data[2] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.yaw_rate_p, &v);
	mr_attitude_control_parameters.rate_p.data[2] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.yaw_rate_i, &v);
	mr_attitude_control_parameters.rate_i.data[2] = v;
	PARAM_GET(mr_attitude_control_parameter_handles.yaw_rate_d, &v);
	mr_attitude_control_parameters.rate_d.data[2] = v;

	PARAM_GET(mr_attitude_control_parameter_handles.yaw_ff, &mr_attitude_control_parameters.yaw_ff);
	PARAM_GET(mr_attitude_control_parameter_handles.rc_scale_yaw, &mr_attitude_control_parameters.rc_scale_yaw);

	return 0;
}


int mr_attitude_control_params_init ()
{
	mr_attitude_control_parameter_handles.roll_p			= 	param_find("MC_ROLL_P");
	mr_attitude_control_parameter_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	mr_attitude_control_parameter_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	mr_attitude_control_parameter_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	mr_attitude_control_parameter_handles.pitch_p			= 	param_find("MC_PITCH_P");
	mr_attitude_control_parameter_handles.pitch_rate_p		= 	param_find("MC_PITCHRATE_P");
	mr_attitude_control_parameter_handles.pitch_rate_i		= 	param_find("MC_PITCHRATE_I");
	mr_attitude_control_parameter_handles.pitch_rate_d		= 	param_find("MC_PITCHRATE_D");
	mr_attitude_control_parameter_handles.yaw_p				=	param_find("MC_YAW_P");
	mr_attitude_control_parameter_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	mr_attitude_control_parameter_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	mr_attitude_control_parameter_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");

	mr_attitude_control_parameter_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	mr_attitude_control_parameter_handles.rc_scale_yaw		= 	param_find("RC_SCALE_YAW");

	return mr_attitude_control_params_update();
}
