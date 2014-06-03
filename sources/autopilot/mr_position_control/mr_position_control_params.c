/**
 * @file mr_position_control_params.c
 *
 * Parameters defined for the mr position control
 *
 */


#include "../../uav_library/param/param.h"
#include "mr_position_control_params.h"


/*
 * Controller parameters
 *
 */
int mr_position_control_params_define ()
{
	/**
	 * Minimum thrust
	 *
	 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
	 *
	 * @min 0.0
	 * @max 1.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_THR_MIN, 0.1f);

	/**
	 * Maximum thrust
	 *
	 * Limit max allowed thrust.
	 *
	 * @min 0.0
	 * @max 1.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_THR_MAX, 0.95f);

	/**
	 * Proportional gain for vertical position error
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_Z_P, 1.0f);

	/**
	 * Proportional gain for vertical velocity error
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_Z_VEL_P, 0.1f);

	/**
	 * Integral gain for vertical velocity error
	 *
	 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_Z_VEL_I, 0.02f);

	/**
	 * Differential gain for vertical velocity error
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_Z_VEL_D, 0.0f);

	/**
	 * Maximum vertical velocity
	 *
	 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (SEATBELT, EASY).
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX, 5.0f);

	/**
	 * Vertical velocity feed forward
	 *
	 * Feed forward weight for altitude control in stabilized modes (SEATBELT, EASY). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
	 *
	 * @min 0.0
	 * @max 1.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_Z_FF, 0.5f);

	/**
	 * Proportional gain for horizontal position error
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_XY_P, 1.0f);

	/**
	 * Proportional gain for horizontal velocity error
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_XY_VEL_P, 0.1f);

	/**
	 * Integral gain for horizontal velocity error
	 *
	 * Non-zero value allows to resist wind.
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_XY_VEL_I, 0.02f);

	/**
	 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_XY_VEL_D, 0.01f);

	/**
	 * Maximum horizontal velocity
	 *
	 * Maximum horizontal velocity in AUTO mode and endpoint for position stabilized mode (EASY).
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 5.0f);

	/**
	 * Horizontal velocity feed forward
	 *
	 * Feed forward weight for position control in position control mode (EASY). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
	 *
	 * @min 0.0
	 * @max 1.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_XY_FF, 0.5f);

	/**
	 * Maximum tilt
	 *
	 * Limits maximum tilt in AUTO and EASY modes.
	 *
	 * @min 0.0
	 * @max 1.57
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_TILT_MAX, 1.0f);

	/**
	 * Landing descend rate
	 *
	 * @min 0.0
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_LAND_SPEED, 1.0f);

	/**
	 * Maximum landing tilt
	 *
	 * Limits maximum tilt on landing.
	 *
	 * @min 0.0
	 * @max 1.57
	 * @group Multicopter Position Control
	 */
	PARAM_DEFINE_FLOAT(MPC_LAND_TILT, 0.3f);

	/**
	 * Roll scaling factor
	 *
	 * @group Radio Calibration
	 */
	PARAM_DEFINE_FLOAT(RC_SCALE_ROLL, 0.6f);

	/**
	 * Pitch scaling factor
	 *
	 * @group Radio Calibration
	 */
	PARAM_DEFINE_FLOAT(RC_SCALE_PITCH, 0.6f);

	return 0;
}

int mr_position_control_params_update()
{
	/*
	struct parameter_update_s param_upd;

	if (orb_check(ORB_ID(parameters_update), params_sub))
		orb_copy(ORB_ID(parameters_update), _params_sub, &param_upd);
	*/

	PARAM_GET(mr_position_control_parameter_handles.thr_min, &mr_position_control_parameters.thr_min);
	PARAM_GET(mr_position_control_parameter_handles.thr_max, &mr_position_control_parameters.thr_max);
	PARAM_GET(mr_position_control_parameter_handles.tilt_max, &mr_position_control_parameters.tilt_max);
	PARAM_GET(mr_position_control_parameter_handles.land_speed, &mr_position_control_parameters.land_speed);
	PARAM_GET(mr_position_control_parameter_handles.land_tilt_max, &mr_position_control_parameters.land_tilt_max);
	PARAM_GET(mr_position_control_parameter_handles.rc_scale_pitch, &mr_position_control_parameters.rc_scale_pitch);
	PARAM_GET(mr_position_control_parameter_handles.rc_scale_roll, &mr_position_control_parameters.rc_scale_roll);

	float v;
	PARAM_GET(mr_position_control_parameter_handles.xy_p, &v);
	mr_position_control_parameters.pos_p.data[0] = v;
	mr_position_control_parameters.pos_p.data[1] = v;
	PARAM_GET(mr_position_control_parameter_handles.z_p, &v);
	mr_position_control_parameters.pos_p.data[2] = v;
	PARAM_GET(mr_position_control_parameter_handles.xy_vel_p, &v);
	mr_position_control_parameters.vel_p.data[0] = v;
	mr_position_control_parameters.vel_p.data[1] = v;
	PARAM_GET(mr_position_control_parameter_handles.z_vel_p, &v);
	mr_position_control_parameters.vel_p.data[2] = v;
	PARAM_GET(mr_position_control_parameter_handles.xy_vel_i, &v);
	mr_position_control_parameters.vel_i.data[0] = v;
	mr_position_control_parameters.vel_i.data[1] = v;
	PARAM_GET(mr_position_control_parameter_handles.z_vel_i, &v);
	mr_position_control_parameters.vel_i.data[2] = v;
	PARAM_GET(mr_position_control_parameter_handles.xy_vel_d, &v);
	mr_position_control_parameters.vel_d.data[0] = v;
	mr_position_control_parameters.vel_d.data[1] = v;
	PARAM_GET(mr_position_control_parameter_handles.z_vel_d, &v);
	mr_position_control_parameters.vel_d.data[2] = v;
	PARAM_GET(mr_position_control_parameter_handles.xy_vel_max, &v);
	mr_position_control_parameters.vel_max.data[0] = v;
	mr_position_control_parameters.vel_max.data[1] = v;
	PARAM_GET(mr_position_control_parameter_handles.z_vel_max, &v);
	mr_position_control_parameters.vel_max.data[2] = v;
	PARAM_GET(mr_position_control_parameter_handles.xy_ff, &v);
	mr_position_control_parameters.vel_ff.data[0] = v;
	mr_position_control_parameters.vel_ff.data[1] = v;
	PARAM_GET(mr_position_control_parameter_handles.z_ff, &v);
	mr_position_control_parameters.vel_ff.data[2] = v;

	MATHLIB_ASSERT (Vector3f_ediv_Vector3f (&mr_position_control_parameters.vel_max, &mr_position_control_parameters.sp_offs_max, &mr_position_control_parameters.pos_p));
	MATHLIB_ASSERT (Vector3f_mul_float (&mr_position_control_parameters.vel_max, 2.0f));

	return 0;
}


int mr_position_control_params_init ()
{
	mr_position_control_parameter_handles.thr_min		= param_find("MPC_THR_MIN");
	mr_position_control_parameter_handles.thr_max		= param_find("MPC_THR_MAX");
	mr_position_control_parameter_handles.z_p			= param_find("MPC_Z_P");
	mr_position_control_parameter_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	mr_position_control_parameter_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	mr_position_control_parameter_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	mr_position_control_parameter_handles.z_vel_max	= param_find("MPC_Z_VEL_MAX");
	mr_position_control_parameter_handles.z_ff		= param_find("MPC_Z_FF");
	mr_position_control_parameter_handles.xy_p		= param_find("MPC_XY_P");
	mr_position_control_parameter_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	mr_position_control_parameter_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	mr_position_control_parameter_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	mr_position_control_parameter_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	mr_position_control_parameter_handles.xy_ff		= param_find("MPC_XY_FF");
	mr_position_control_parameter_handles.tilt_max	= param_find("MPC_TILT_MAX");
	mr_position_control_parameter_handles.land_speed	= param_find("MPC_LAND_SPEED");
	mr_position_control_parameter_handles.land_tilt_max	= param_find("MPC_LAND_TILT");
	mr_position_control_parameter_handles.rc_scale_pitch	= param_find("RC_SCALE_PITCH");
	mr_position_control_parameter_handles.rc_scale_roll	= param_find("RC_SCALE_ROLL");

	return mr_position_control_params_update();
}
