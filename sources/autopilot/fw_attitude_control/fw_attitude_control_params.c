/**
 * @file fw_attitude_control_params.c
 *
 * Parameters defined for the attitude controller
 *
 */


#include "../../uav_library/param/param.h"
#include "../../uav_library/math/limits.h"

#include "fw_attitude_control_params.h"
#include "ECL_roll_controller.h"
#include "ECL_pitch_controller.h"
#include "ECL_yaw_controller.h"


/*
 * Controller parameters
 *
 */
int fw_attitude_control_params_define ()
{
	// @DisplayName		Attitude Time Constant
	// @Description		This defines the latency between a step input and the achieved setpoint. Half a second is a good start value and fits for most average systems. Smaller systems may require smaller values, but as this will wear out servos faster, the value should only be decreased as needed.
	// @Range		0.4 to 1.0 seconds, in tens of seconds
	PARAM_DEFINE_FLOAT (FW_ATT_TC, 0.5f);

	// @DisplayName		Proportional gain.
	// @Description		This defines how much the elevator input will be commanded dependend on the current pitch error.
	// @Range		10 to 200, 1 increments
	PARAM_DEFINE_FLOAT (FW_P_P, 40.0f);

	// @DisplayName		Damping gain.
	// @Description		This gain damps the airframe pitch rate. In particular relevant for flying wings.
	// @Range		0.0 to 10.0, 0.1 increments
	PARAM_DEFINE_FLOAT (FW_P_D, 0.0f);

	// @DisplayName		Integrator gain.
	// @Description		This gain defines how much control response will result out of a steady state error. It trims any constant error.
	// @Range		0 to 50.0
	PARAM_DEFINE_FLOAT (FW_P_I, 0.0f);

	// @DisplayName		Maximum positive / up pitch rate.
	// @Description		This limits the maximum pitch up angular rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
	// @Range		0 to 90.0 degrees per seconds, in 1 increments
	PARAM_DEFINE_FLOAT (FW_P_RMAX_POS, 0.0f);

	// @DisplayName		Maximum negative / down pitch rate.
	// @Description		This limits the maximum pitch down up angular rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
	// @Range		0 to 90.0 degrees per seconds, in 1 increments
	PARAM_DEFINE_FLOAT (FW_P_RMAX_NEG, 0.0f);

	// @DisplayName		Pitch Integrator Anti-Windup
	// @Description		This limits the range in degrees the integrator can wind up to.
	// @Range		0.0 to 45.0
	// @Increment		1.0
	PARAM_DEFINE_FLOAT (FW_P_IMAX, 15.0f);

	// @DisplayName		Roll feedforward gain.
	// @Description		This compensates during turns and ensures the nose stays level.
	// @Range		0.5 2.0
	// @Increment		0.05
	// @User		User
	PARAM_DEFINE_FLOAT (FW_P_ROLLFF, 1.0f);

	// @DisplayName		Proportional Gain.
	// @Description		This gain controls the roll angle to roll actuator output.
	// @Range		10.0 200.0
	// @Increment		10.0
	// @User		User
	PARAM_DEFINE_FLOAT (FW_R_P, 40.0f);

	// @DisplayName		Damping Gain
	// @Description		Controls the roll rate to roll actuator output. It helps to reduce motions in turbulence.
	// @Range		0.0 10.0
	// @Increment		1.0
	// @User		User
	PARAM_DEFINE_FLOAT (FW_R_D, 0.0f);

	// @DisplayName		Integrator Gain
	// @Description		This gain controls the contribution of the integral to roll actuator outputs. It trims out steady state errors.
	// @Range		0.0 100.0
	// @Increment		5.0
	// @User		User
	PARAM_DEFINE_FLOAT (FW_R_I, 0.0f);

	// @DisplayName		Roll Integrator Anti-Windup
	// @Description		This limits the range in degrees the integrator can wind up to.
	// @Range		0.0 to 45.0
	// @Increment		1.0
	PARAM_DEFINE_FLOAT (FW_R_IMAX, 15.0f);

	// @DisplayName		Maximum Roll Rate
	// @Description		This limits the maximum roll rate the controller will output (in degrees per second). Setting a value of zero disables the limit.
	// @Range		0 to 90.0 degrees per seconds
	// @Increment		1.0
	PARAM_DEFINE_FLOAT (FW_R_RMAX, 60);


	PARAM_DEFINE_FLOAT (FW_Y_P, 0);
	PARAM_DEFINE_FLOAT (FW_Y_I, 0);
	PARAM_DEFINE_FLOAT (FW_Y_IMAX, 15.0f);
	PARAM_DEFINE_FLOAT (FW_Y_D, 0);
	PARAM_DEFINE_FLOAT (FW_Y_ROLLFF, 1);
	PARAM_DEFINE_FLOAT (FW_AIRSPD_MIN, 9.0f);
	PARAM_DEFINE_FLOAT (FW_AIRSPD_TRIM, 12.0f);
	PARAM_DEFINE_FLOAT (FW_AIRSPD_MAX, 18.0f);

	return 0;
}

int fw_attitude_control_parameters_update()
{
	PARAM_GET(_fw_attitude_control_parameter_handles.tconst, &(_fw_attitude_control_parameters.tconst));
	PARAM_GET(_fw_attitude_control_parameter_handles.p_p, &(_fw_attitude_control_parameters.p_p));
	PARAM_GET(_fw_attitude_control_parameter_handles.p_d, &(_fw_attitude_control_parameters.p_d));
	PARAM_GET(_fw_attitude_control_parameter_handles.p_i, &(_fw_attitude_control_parameters.p_i));
	PARAM_GET(_fw_attitude_control_parameter_handles.p_rmax_pos, &(_fw_attitude_control_parameters.p_rmax_pos));
	PARAM_GET(_fw_attitude_control_parameter_handles.p_rmax_neg, &(_fw_attitude_control_parameters.p_rmax_neg));
	PARAM_GET(_fw_attitude_control_parameter_handles.p_integrator_max, &(_fw_attitude_control_parameters.p_integrator_max));
	PARAM_GET(_fw_attitude_control_parameter_handles.p_roll_feedforward, &(_fw_attitude_control_parameters.p_roll_feedforward));

	PARAM_GET(_fw_attitude_control_parameter_handles.r_p, &(_fw_attitude_control_parameters.r_p));
	PARAM_GET(_fw_attitude_control_parameter_handles.r_d, &(_fw_attitude_control_parameters.r_d));
	PARAM_GET(_fw_attitude_control_parameter_handles.r_i, &(_fw_attitude_control_parameters.r_i));
	PARAM_GET(_fw_attitude_control_parameter_handles.r_integrator_max, &(_fw_attitude_control_parameters.r_integrator_max));
	PARAM_GET(_fw_attitude_control_parameter_handles.r_rmax, &(_fw_attitude_control_parameters.r_rmax));

	PARAM_GET(_fw_attitude_control_parameter_handles.y_p, &(_fw_attitude_control_parameters.y_p));
	PARAM_GET(_fw_attitude_control_parameter_handles.y_i, &(_fw_attitude_control_parameters.y_i));
	PARAM_GET(_fw_attitude_control_parameter_handles.y_d, &(_fw_attitude_control_parameters.y_d));
	PARAM_GET(_fw_attitude_control_parameter_handles.y_roll_feedforward, &(_fw_attitude_control_parameters.y_roll_feedforward));
	PARAM_GET(_fw_attitude_control_parameter_handles.y_integrator_max, &(_fw_attitude_control_parameters.y_integrator_max));

	PARAM_GET(_fw_attitude_control_parameter_handles.airspeed_min, &(_fw_attitude_control_parameters.airspeed_min));
	PARAM_GET(_fw_attitude_control_parameter_handles.airspeed_trim, &(_fw_attitude_control_parameters.airspeed_trim));
	PARAM_GET(_fw_attitude_control_parameter_handles.airspeed_max, &(_fw_attitude_control_parameters.airspeed_max));

	/* pitch control parameters */
	ECL_pitch_controller_set_time_constant(_fw_attitude_control_parameters.tconst);
	ECL_pitch_controller_set_k_p(radians(_fw_attitude_control_parameters.p_p));
	ECL_pitch_controller_set_k_i(radians(_fw_attitude_control_parameters.p_i));
	ECL_pitch_controller_set_k_d(radians(_fw_attitude_control_parameters.p_d));
	ECL_pitch_controller_set_integrator_max(radians(_fw_attitude_control_parameters.p_integrator_max));
	ECL_pitch_controller_set_max_rate_pos(radians(_fw_attitude_control_parameters.p_rmax_pos));
	ECL_pitch_controller_set_max_rate_neg(radians(_fw_attitude_control_parameters.p_rmax_neg));
	ECL_pitch_controller_set_roll_ff(radians(_fw_attitude_control_parameters.p_roll_feedforward));

	/* roll control parameters */
	ECL_roll_controller_set_time_constant(_fw_attitude_control_parameters.tconst);
	ECL_roll_controller_set_k_p(radians(_fw_attitude_control_parameters.r_p));
	ECL_roll_controller_set_k_i(radians(_fw_attitude_control_parameters.r_i));
	ECL_roll_controller_set_k_d(radians(_fw_attitude_control_parameters.r_d));
	ECL_roll_controller_set_integrator_max(radians(_fw_attitude_control_parameters.r_integrator_max));
	ECL_roll_controller_set_max_rate(radians(_fw_attitude_control_parameters.r_rmax));

	/* yaw control parameters */
	ECL_yaw_controller_set_k_side(radians(_fw_attitude_control_parameters.y_p));
	ECL_yaw_controller_set_k_i(radians(_fw_attitude_control_parameters.y_i));
	ECL_yaw_controller_set_k_d(radians(_fw_attitude_control_parameters.y_d));
	ECL_yaw_controller_set_k_roll_ff(radians(_fw_attitude_control_parameters.y_roll_feedforward));
	ECL_yaw_controller_set_integrator_max(radians(_fw_attitude_control_parameters.y_integrator_max));

	return 0;
}


int fw_attitude_control_param_init ()
{
	_fw_attitude_control_parameter_handles.tconst = param_find("FW_ATT_TC");
	_fw_attitude_control_parameter_handles.p_p = param_find("FW_P_P");
	_fw_attitude_control_parameter_handles.p_d = param_find("FW_P_D");
	_fw_attitude_control_parameter_handles.p_i = param_find("FW_P_I");
	_fw_attitude_control_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_fw_attitude_control_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_fw_attitude_control_parameter_handles.p_integrator_max = param_find("FW_P_INTEGR_MAX");
	_fw_attitude_control_parameter_handles.p_roll_feedforward = param_find("FW_P_ROLLFF");

	_fw_attitude_control_parameter_handles.r_p = param_find("FW_R_P");
	_fw_attitude_control_parameter_handles.r_d = param_find("FW_R_D");
	_fw_attitude_control_parameter_handles.r_i = param_find("FW_R_I");
	_fw_attitude_control_parameter_handles.r_integrator_max = param_find("FW_R_INTEGR_MAX");
	_fw_attitude_control_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_fw_attitude_control_parameter_handles.y_p = param_find("FW_Y_P");
	_fw_attitude_control_parameter_handles.y_i = param_find("FW_Y_I");
	_fw_attitude_control_parameter_handles.y_d = param_find("FW_Y_D");
	_fw_attitude_control_parameter_handles.y_roll_feedforward = param_find("FW_Y_ROLLFF");
	_fw_attitude_control_parameter_handles.y_integrator_max = param_find("FW_Y_INTEGR_MAX");

	_fw_attitude_control_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_fw_attitude_control_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_fw_attitude_control_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	/* fetch initial parameter values */
	return fw_attitude_control_parameters_update();
}
