/**
 * @file fw_position_control_params.c
 *
 * Parameters defined for the L1 position control
 *
 */


#include "../../uav_library/param/param.h"
#include "../../uav_library/math/limits.h"

#include "fw_position_control_params.h"
#include "ECL_l1_position_controller.h"
#include "ECL_tecs.h"


/*
 * Controller parameters
 *
 */
int fw_position_control_params_define ()
{
	PARAM_DEFINE_FLOAT (FW_L1_PERIOD, 25.0f);
	PARAM_DEFINE_FLOAT (FW_L1_DAMPING, 0.75f);
	PARAM_DEFINE_FLOAT (FW_LOITER_R, 50.0f);
	PARAM_DEFINE_FLOAT (FW_THR_CRUISE, 0.7f);
	PARAM_DEFINE_FLOAT (FW_P_LIM_MIN, -45.0f);
	PARAM_DEFINE_FLOAT (FW_P_LIM_MAX, 45.0f);
	PARAM_DEFINE_FLOAT (FW_R_LIM, 45.0f);
	PARAM_DEFINE_FLOAT (FW_THR_MIN, 0.0f);
	PARAM_DEFINE_FLOAT (FW_THR_MAX, 1.0f);
	PARAM_DEFINE_FLOAT (FW_THR_LND_MAX, 1.0f);
	PARAM_DEFINE_FLOAT (FW_T_CLMB_MAX, 5.0f);
	PARAM_DEFINE_FLOAT (FW_T_SINK_MIN, 2.0f);
	PARAM_DEFINE_FLOAT (FW_T_TIME_CONST, 5.0f);
	PARAM_DEFINE_FLOAT (FW_T_THR_DAMP, 0.5f);
	PARAM_DEFINE_FLOAT (FW_T_INTEG_GAIN, 0.1f);
	PARAM_DEFINE_FLOAT (FW_T_VERT_ACC, 7.0f);
	PARAM_DEFINE_FLOAT (FW_T_HGT_OMEGA, 3.0f);
	PARAM_DEFINE_FLOAT (FW_T_SPD_OMEGA, 2.0f);
	PARAM_DEFINE_FLOAT (FW_T_RLL2THR, 10.0f);
	PARAM_DEFINE_FLOAT (FW_T_SPDWEIGHT, 1.0f);
	PARAM_DEFINE_FLOAT (FW_T_PTCH_DAMP, 0.0f);
	PARAM_DEFINE_FLOAT (FW_T_SINK_MAX, 5.0f);

	return 0;
}

int fw_position_control_parameters_update()
{
	/* L1 control parameters */
	PARAM_GET(_fw_position_control_parameter_handles.l1_damping, &(_fw_position_control_parameters.l1_damping));
	PARAM_GET(_fw_position_control_parameter_handles.l1_period, &(_fw_position_control_parameters.l1_period));
	PARAM_GET(_fw_position_control_parameter_handles.loiter_hold_radius, &(_fw_position_control_parameters.loiter_hold_radius));

	PARAM_GET(_fw_position_control_parameter_handles.airspeed_min, &(_fw_position_control_parameters.airspeed_min));
	PARAM_GET(_fw_position_control_parameter_handles.airspeed_trim, &(_fw_position_control_parameters.airspeed_trim));
	PARAM_GET(_fw_position_control_parameter_handles.airspeed_max, &(_fw_position_control_parameters.airspeed_max));

	PARAM_GET(_fw_position_control_parameter_handles.pitch_limit_min, &(_fw_position_control_parameters.pitch_limit_min));
	PARAM_GET(_fw_position_control_parameter_handles.pitch_limit_max, &(_fw_position_control_parameters.pitch_limit_max));
	PARAM_GET(_fw_position_control_parameter_handles.roll_limit, &(_fw_position_control_parameters.roll_limit));
	PARAM_GET(_fw_position_control_parameter_handles.throttle_min, &(_fw_position_control_parameters.throttle_min));
	PARAM_GET(_fw_position_control_parameter_handles.throttle_max, &(_fw_position_control_parameters.throttle_max));
	PARAM_GET(_fw_position_control_parameter_handles.throttle_cruise, &(_fw_position_control_parameters.throttle_cruise));

	PARAM_GET(_fw_position_control_parameter_handles.throttle_land_max, &(_fw_position_control_parameters.throttle_land_max));

	PARAM_GET(_fw_position_control_parameter_handles.time_const, &(_fw_position_control_parameters.time_const));
	PARAM_GET(_fw_position_control_parameter_handles.min_sink_rate, &(_fw_position_control_parameters.min_sink_rate));
	PARAM_GET(_fw_position_control_parameter_handles.max_sink_rate, &(_fw_position_control_parameters.max_sink_rate));
	PARAM_GET(_fw_position_control_parameter_handles.throttle_damp, &(_fw_position_control_parameters.throttle_damp));
	PARAM_GET(_fw_position_control_parameter_handles.integrator_gain, &(_fw_position_control_parameters.integrator_gain));
	PARAM_GET(_fw_position_control_parameter_handles.vertical_accel_limit, &(_fw_position_control_parameters.vertical_accel_limit));
	PARAM_GET(_fw_position_control_parameter_handles.height_comp_filter_omega, &(_fw_position_control_parameters.height_comp_filter_omega));
	PARAM_GET(_fw_position_control_parameter_handles.speed_comp_filter_omega, &(_fw_position_control_parameters.speed_comp_filter_omega));
	PARAM_GET(_fw_position_control_parameter_handles.roll_throttle_compensation, &(_fw_position_control_parameters.roll_throttle_compensation));
	PARAM_GET(_fw_position_control_parameter_handles.speed_weight, &(_fw_position_control_parameters.speed_weight));
	PARAM_GET(_fw_position_control_parameter_handles.pitch_damping, &(_fw_position_control_parameters.pitch_damping));
	PARAM_GET(_fw_position_control_parameter_handles.max_climb_rate, &(_fw_position_control_parameters.max_climb_rate));

	ECL_l1_position_controller_set_l1_damping(_fw_position_control_parameters.l1_damping);
	ECL_l1_position_controller_set_l1_period(_fw_position_control_parameters.l1_period);
	ECL_l1_position_controller_set_l1_roll_limit(radians(_fw_position_control_parameters.roll_limit));

	ECL_tecs_set_time_const(_fw_position_control_parameters.time_const);
	ECL_tecs_set_min_sink_rate(_fw_position_control_parameters.min_sink_rate);
	ECL_tecs_set_max_sink_rate(_fw_position_control_parameters.max_sink_rate);
	ECL_tecs_set_throttle_damp(_fw_position_control_parameters.throttle_damp);
	ECL_tecs_set_integrator_gain(_fw_position_control_parameters.integrator_gain);
	ECL_tecs_set_vertical_accel_limit(_fw_position_control_parameters.vertical_accel_limit);
	ECL_tecs_set_height_comp_filter_omega(_fw_position_control_parameters.height_comp_filter_omega);
	ECL_tecs_set_speed_comp_filter_omega(_fw_position_control_parameters.speed_comp_filter_omega);
	ECL_tecs_set_roll_throttle_compensation(radians(_fw_position_control_parameters.roll_throttle_compensation));
	ECL_tecs_set_speed_weight(_fw_position_control_parameters.speed_weight);
	ECL_tecs_set_pitch_damping(_fw_position_control_parameters.pitch_damping);
	ECL_tecs_set_indicated_airspeed_min(_fw_position_control_parameters.airspeed_min);
	ECL_tecs_set_indicated_airspeed_max(_fw_position_control_parameters.airspeed_min);
	ECL_tecs_set_max_climb_rate(_fw_position_control_parameters.max_climb_rate);

	/* sanity check parameters */
	if (_fw_position_control_parameters.airspeed_max < _fw_position_control_parameters.airspeed_min ||
		_fw_position_control_parameters.airspeed_max < 5.0f ||
		_fw_position_control_parameters.airspeed_min > 100.0f ||
		_fw_position_control_parameters.airspeed_trim < _fw_position_control_parameters.airspeed_min ||
		_fw_position_control_parameters.airspeed_trim > _fw_position_control_parameters.airspeed_max) {
		//warnx("error: airspeed parameters invalid");
		return -1;
	}

	return 0;
}


int fw_position_control_param_init ()
{
	_fw_position_control_parameter_handles.l1_period = param_find("FW_L1_PERIOD");
	_fw_position_control_parameter_handles.l1_damping = param_find("FW_L1_DAMPING");
	_fw_position_control_parameter_handles.loiter_hold_radius = param_find("FW_LOITER_R");

	_fw_position_control_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_fw_position_control_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_fw_position_control_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_fw_position_control_parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
	_fw_position_control_parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
	_fw_position_control_parameter_handles.roll_limit = param_find("FW_R_LIM");
	_fw_position_control_parameter_handles.throttle_min = param_find("FW_THR_MIN");
	_fw_position_control_parameter_handles.throttle_max = param_find("FW_THR_MAX");
	_fw_position_control_parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
	_fw_position_control_parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");

	_fw_position_control_parameter_handles.time_const = param_find("FW_T_TIME_CONST");
	_fw_position_control_parameter_handles.min_sink_rate = param_find("FW_T_SINK_MIN");
	_fw_position_control_parameter_handles.max_sink_rate = param_find("FW_T_SINK_MAX");
	_fw_position_control_parameter_handles.max_climb_rate = param_find("FW_T_CLMB_MAX");
	_fw_position_control_parameter_handles.throttle_damp = param_find("FW_T_THR_DAMP");
	_fw_position_control_parameter_handles.integrator_gain = param_find("FW_T_INTEG_GAIN");
	_fw_position_control_parameter_handles.vertical_accel_limit = param_find("FW_T_VERT_ACC");
	_fw_position_control_parameter_handles.height_comp_filter_omega = param_find("FW_T_HGT_OMEGA");
	_fw_position_control_parameter_handles.speed_comp_filter_omega = param_find("FW_T_SPD_OMEGA");
	_fw_position_control_parameter_handles.roll_throttle_compensation = param_find("FW_T_RLL2THR");
	_fw_position_control_parameter_handles.speed_weight = param_find("FW_T_SPDWEIGHT");
	_fw_position_control_parameter_handles.pitch_damping = param_find("FW_T_PTCH_DAMP");

	/* fetch initial parameter values */
	return fw_position_control_parameters_update();
}
