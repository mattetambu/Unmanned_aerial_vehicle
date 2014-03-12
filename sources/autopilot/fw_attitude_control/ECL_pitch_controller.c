/**
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include <math.h>
#include <stdint.h>

#include "ECL_pitch_controller.h"
#include "../../uav_library/common.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/math/limits.h"


struct ECL_pitch_controller {
	absolute_time _last_run;
	float _tc;
	float _k_p;
	float _k_i;
	float _k_d;
	float _integrator_max;
	float _max_rate_pos;
	float _max_rate_neg;
	float _roll_ff;
	float _last_output;
	float _integrator;
	float _rate_error;
	float _rate_setpoint;
	float _max_deflection_rad;
} epc;


void ECL_pitch_controller_init ()
{
	epc._last_run = 0;
	epc._tc = 0.1f;
	epc._last_output = 0.0f;
	epc._integrator = 0.0f;
	epc._rate_error = 0.0f;
	epc._rate_setpoint = 0.0f;
	epc._max_deflection_rad = radians(45.0f);
}

void ECL_pitch_controller_reset_integrator()
{
	epc._integrator = 0.0f;
}

void ECL_pitch_controller_set_time_constant(float time_constant) {
	epc._tc = time_constant;
}
void ECL_pitch_controller_set_k_p(float k_p) {
	epc._k_p = k_p;
}
void ECL_pitch_controller_set_k_i(float k_i) {
	epc._k_i = k_i;
}
void ECL_pitch_controller_set_k_d(float k_d) {
	epc._k_d = k_d;
}
void ECL_pitch_controller_set_integrator_max(float max) {
	epc._integrator_max = max;
}
void ECL_pitch_controller_set_max_rate_pos(float max_rate_pos) {
	epc._max_rate_pos = max_rate_pos;
}
void ECL_pitch_controller_set_max_rate_neg(float max_rate_neg) {
	epc._max_rate_neg = max_rate_neg;
}
void ECL_pitch_controller_set_roll_ff(float roll_ff) {
	epc._roll_ff = roll_ff;
}

float ECL_pitch_controller_get_rate_error() {
	return epc._rate_error;
}

float ECL_pitch_controller_get_desired_rate() {
	return epc._rate_setpoint;
}


float ECL_pitch_controller_control(float pitch_setpoint, float pitch, float pitch_rate, float roll, float scaler,
				   bool_t lock_integrator, float airspeed_min, float airspeed_max, float airspeed)
{
	/* get the usual dt estimate */
	uint64_t dt_micros = absolute_elapsed_time(&epc._last_run);
	epc._last_run = get_absolute_time();

	float dt = dt_micros / 1000000;

	/* lock integral for long intervals */
	if (dt_micros > 500000)
		lock_integrator = 1;

	float k_roll_ff = max((epc._k_p - epc._k_i * epc._tc) * epc._tc - epc._k_d, 0.0f);
	float k_i_rate = epc._k_i * epc._tc;

	/* input conditioning */
	//airspeed = constrain (airspeed, airspeed_min, airspeed_max);
	if (!check_finite(airspeed)) {
		/* airspeed is NaN, +- INF or not available, pick center of band */
		airspeed = 0.5f * (airspeed_min + airspeed_max);
	} else if (airspeed < airspeed_min) {
		airspeed = airspeed_min;
	}

	/* flying inverted (wings upside down) ? */
	bool_t inverted = 0;

	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(roll) < radians(90.0f)) {
		/* not inverted, but numerically still potentially close to infinity */
		roll = constrain(roll, radians(-80.0f), radians(80.0f));
	} else {
		/* inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity */

		/* note: the ranges are extended by 10 deg here to avoid numeric resolution effects */
		if (roll > 0.0f) {
			/* right hemisphere */
			roll = constrain(roll, radians(100.0f), radians(180.0f));
		} else {
			/* left hemisphere */
			roll = constrain(roll, radians(-100.0f), radians(-180.0f));
		}
	}

	/* calculate the offset in the rate resulting from rolling  */
	float turn_offset = fabsf((CONSTANTS_ONE_G / airspeed) *
				tanf(roll) * sinf(roll)) * epc._roll_ff;
	if (inverted)
		turn_offset = -turn_offset;

	float pitch_error = pitch_setpoint - pitch;
	/* rate setpoint from current error and time constant */
	epc._rate_setpoint = pitch_error / epc._tc;

	/* add turn offset */
	epc._rate_setpoint += turn_offset;

	epc._rate_error = epc._rate_setpoint - pitch_rate;

	float ilimit_scaled = 0.0f;

	if (!lock_integrator && k_i_rate > 0.0f && airspeed > 0.5f * airspeed_min) {

		float id = epc._rate_error * k_i_rate * dt * scaler;

		/*
		 * anti-windup: do not allow integrator to increase into the
		 * wrong direction if actuator is at limit
		 */
		if (epc._last_output < -epc._max_deflection_rad) {
			/* only allow motion to center: increase value */
			id = max(id, 0.0f);
		} else if (epc._last_output > epc._max_deflection_rad) {
			/* only allow motion to center: decrease value */
			id = min(id, 0.0f);
		}

		epc._integrator += id;
	}

	/* integrator limit */
	epc._integrator = constrain(epc._integrator, -ilimit_scaled, ilimit_scaled);
	/* store non-limited output */
	epc._last_output = ((epc._rate_error * epc._k_d * scaler) + epc._integrator + (epc._rate_setpoint * k_roll_ff)) * scaler;

	return constrain(epc._last_output, -epc._max_deflection_rad, epc._max_deflection_rad);
}
