/**
 * @file attitude_estimator_so3_comp_params.c
 *
 * Parameters defined for attitude estimator
 *
 */


#include "../../uav_library/param/param.h"
#include "attitude_estimator_so3_comp_params.h"


/*
 * This is filter gain for nonlinear SO3 complementary filter
 */
/*
 * Controller parameters
 * NOTE : How to tune the gain? First of all, stick with this default gain. And let the quad in stable place.
 * Log the steady state reponse of filter. If it is too slow, increase SO3_COMP_KP.
 * If you are flying from ground to high altitude in short amount of time, please increase SO3_COMP_KI which
 * will compensate gyro bias which depends on temperature and vibration of your vehicle
 */
int non_linear_SO3_AHRS_comp_params_define ()
{
	PARAM_DEFINE_FLOAT(SO3_COMP_KP, 1.0f); //! This parameter will give you about 15 seconds convergence time.
										   //! You can set this gain higher if you want more fast response.
										   //! But note that higher gain will give you also higher overshoot.
	PARAM_DEFINE_FLOAT(SO3_COMP_KI, 0.05f); //! This gain will incorporate slow time-varying bias (e.g., temperature change)
											//! This gain is depend on your vehicle status.

	/* offsets in roll, pitch and yaw of sensor plane and body */
	PARAM_DEFINE_FLOAT(ATT_ROLL_OFFS, 0.0f);
	PARAM_DEFINE_FLOAT(ATT_PITCH_OFFS, 0.0f);
	PARAM_DEFINE_FLOAT(ATT_YAW_OFFS, 0.0f);

	return 0;
}

/**
 * Update all parameters
 *
 */
int non_linear_SO3_AHRS_comp_params_update ()
{
	/* Update filter gain */
	PARAM_GET(so3_comp_param_handles.Kp, &(so3_comp_params.Kp));
	PARAM_GET(so3_comp_param_handles.Ki, &(so3_comp_params.Ki));

	/* Update attitude offset */
	PARAM_GET(so3_comp_param_handles.roll_off, &(so3_comp_params.roll_off));
	PARAM_GET(so3_comp_param_handles.pitch_off, &(so3_comp_params.pitch_off));
	PARAM_GET(so3_comp_param_handles.yaw_off, &(so3_comp_params.yaw_off));

	return 0;
}

/**
 * Initialize all parameter handles and values
 *
 */
int non_linear_SO3_AHRS_comp_params_init ()
{
	/* Filter gain parameters */
	so3_comp_param_handles.Kp = param_find("SO3_COMP_KP");
	so3_comp_param_handles.Ki = param_find("SO3_COMP_KI");

	/* Attitude offset (WARNING: Do not change if you do not know what exactly this variable wil lchange) */
	so3_comp_param_handles.roll_off = param_find("ATT_ROLL_OFFS");
	so3_comp_param_handles.pitch_off = param_find("ATT_PITCH_OFFS");
	so3_comp_param_handles.yaw_off = param_find("ATT_YAW_OFFS");

	return non_linear_SO3_AHRS_comp_params_update();
}
