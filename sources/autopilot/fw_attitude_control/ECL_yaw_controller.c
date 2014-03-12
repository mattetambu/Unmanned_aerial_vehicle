/**
 * @file ecl_yaw_controller.cpp
 * Implementation of a simple orthogonal coordinated turn yaw PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include <math.h>
#include <stdint.h>

#include "ECL_yaw_controller.h"
#include "../../uav_library/common.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/math/limits.h"


struct ECL_yaw_controller {
	absolute_time _last_run;
	float _k_side;
	float _k_i;
	float _k_d;
	float _k_roll_ff;
	float _integrator_max;

	float _last_error;
	float _last_output;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _k_d_last;
	float _integrator;
} eyc;


void ECL_yaw_controller_init ()
{
	eyc._last_run = 0;
	eyc._last_error = 0.0f;
	eyc._last_output = 0.0f;
	eyc._last_rate_hp_out = 0.0f;
	eyc._last_rate_hp_in = 0.0f;
	eyc._k_d_last = 0.0f;
	eyc._integrator = 0.0f;
}

void ECL_yaw_controller_reset_integrator()
{
	eyc._integrator = 0.0f;
}

void ECL_yaw_controller_set_k_side(float k_a) {
	eyc._k_side = k_a;
}
void ECL_yaw_controller_set_k_i(float k_i) {
	eyc._k_i = k_i;
}
void ECL_yaw_controller_set_k_d(float k_d) {
	eyc._k_d = k_d;
}
void ECL_yaw_controller_set_k_roll_ff(float k_roll_ff) {
	eyc._k_roll_ff = k_roll_ff;
}
void ECL_yaw_controller_set_integrator_max(float max) {
	eyc._integrator_max = max;
}


float ECL_yaw_controller_control(float roll, float yaw_rate, float accel_y, float scaler, bool_t lock_integrator,
				 float airspeed_min, float airspeed_max, float aspeed)
{
	// XXX NOT YET IMPLEMENTED

	/* get the usual dt estimate */
	//absolute_time dt_micros = absolute_elapsed_time(&eyc._last_run);
	//eyc._last_run = get_absolute_time();

	//float dt = (dt_micros > 500000) ? 0.0f : dt_micros / 1000000;

	return 0.0f;
}
