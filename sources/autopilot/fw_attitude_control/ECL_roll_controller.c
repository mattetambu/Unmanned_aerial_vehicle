/**
 * @file ecl_roll_controller.cpp
 * Implementation of a simple orthogonal roll PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include <math.h>
#include <stdint.h>

#include "ECL_roll_controller.h"
#include "../../uav_library/common.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/math/limits.h"


struct ECL_roll_controller {
	absolute_time _last_run;
	float _tc;
	float _k_p;
	float _k_i;
	float _k_d;
	float _integrator_max;
	float _max_rate;
	float _last_output;
	float _integrator;
	float _rate_error;
	float _rate_setpoint;
	float _max_deflection_rad;
} erc;


void ECL_roll_controller_init ()
{
	erc._last_run = 0;
	erc._tc = 0.1f;
	erc._last_output = 0.0f;
	erc._integrator = 0.0f;
	erc._rate_error = 0.0f;
	erc._rate_setpoint = 0.0f;
	erc._max_deflection_rad = radians(45.0f);
}

void ECL_roll_controller_reset_integrator()
{
	erc._integrator = 0.0f;
}

void ECL_roll_controller_set_time_constant(float time_constant) {
	if (time_constant > 0.1f && time_constant < 3.0f) {
		erc._tc = time_constant;
	}
}
void ECL_roll_controller_set_k_p(float k_p) {
	erc._k_p = k_p;
}
void ECL_roll_controller_set_k_i(float k_i) {
	erc._k_i = k_i;
}
void ECL_roll_controller_set_k_d(float k_d) {
	erc._k_d = k_d;
}
void ECL_roll_controller_set_integrator_max(float max) {
	erc._integrator_max = max;
}
void ECL_roll_controller_set_max_rate(float max_rate) {
	erc._max_rate = max_rate;
}

float ECL_roll_controller_get_rate_error() {
	return erc._rate_error;
}

float ECL_roll_controller_get_desired_rate() {
	return erc._rate_setpoint;
}


float ECL_roll_controller_control(float roll_setpoint, float roll, float roll_rate,
				  float scaler, bool_t lock_integrator, float airspeed_min, float airspeed_max, float airspeed)
{
	/* get the usual dt estimate */
	uint64_t dt_micros = absolute_elapsed_time(&erc._last_run);
	erc._last_run = get_absolute_time();

	float dt = (dt_micros > 500000) ? 0.0f : dt_micros / 1000000;

	float k_ff = max((erc._k_p - erc._k_i * erc._tc) * erc._tc - erc._k_d, 0.0f);
	float k_i_rate = erc._k_i * erc._tc;

	/* input conditioning */
	//airspeed = constrain (airspeed, airspeed_min, airspeed_max);
	if (!check_finite(airspeed)) {
		/* airspeed is NaN, +- INF or not available, pick center of band */
		airspeed = 0.5f * (airspeed_min + airspeed_max);
	} else if (airspeed < airspeed_min) {
		airspeed = airspeed_min;
	}

	float roll_error = roll_setpoint - roll;
	erc._rate_setpoint = roll_error / erc._tc;

	/* limit the rate */
	if (erc._max_rate > 0.01f) {
		erc._rate_setpoint = (erc._rate_setpoint > erc._max_rate) ? erc._max_rate : erc._rate_setpoint;
		erc._rate_setpoint = (erc._rate_setpoint < -erc._max_rate) ? -erc._max_rate : erc._rate_setpoint;
	}

	erc._rate_error = erc._rate_setpoint - roll_rate;


	float ilimit_scaled = 0.0f;

	if (!lock_integrator && k_i_rate > 0.0f && airspeed > 0.5f * airspeed_min) {

		float id = erc._rate_error * k_i_rate * dt * scaler;

		/*
		 * anti-windup: do not allow integrator to increase into the
		 * wrong direction if actuator is at limit
		 */
		if (erc._last_output < -erc._max_deflection_rad) {
			/* only allow motion to center: increase value */
			id = max(id, 0.0f);
		} else if (erc._last_output > erc._max_deflection_rad) {
			/* only allow motion to center: decrease value */
			id = min(id, 0.0f);
		}

		erc._integrator += id;
	}

	/* integrator limit */
	erc._integrator = constrain(erc._integrator, -ilimit_scaled, ilimit_scaled);
	/* store non-limited output */
	erc._last_output = ((erc._rate_error * erc._k_d * scaler) + erc._integrator + (erc._rate_setpoint * k_ff)) * scaler;

	return constrain(erc._last_output, -erc._max_deflection_rad, erc._max_deflection_rad);
}
