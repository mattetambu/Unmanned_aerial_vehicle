/**
 * @file multirotor_rate_control.c
 *
 * Implementation of rate controller for multirotors.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "../../uav_library/common.h"
#include "../../uav_library/pid/pid.h"
#include "../../uav_library/math/limits.h"
#include "../../uav_library/time/drv_time.h"

#include "multirotor_rate_control_params.h"
#include "multirotor_rate_control.h"


static absolute_time last_run = 0;
static absolute_time last_input = 0;
static int motor_skip_counter = 0;
static bool_t initialized = 0 /* false */;

static PID_t pitch_rate_controller;
static PID_t roll_rate_controller;


void multirotor_control_rates(const struct vehicle_rates_setpoint_s *rate_sp,
			      const float rates[], struct actuator_controls_s *actuators, bool_t reset_integral)
{
	const float deltaT = (get_absolute_time() - last_run) / 1000000.0f;
	last_run = get_absolute_time();

	if (last_input != rate_sp->timestamp) {
		last_input = rate_sp->timestamp;
	}


	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == 0 /* false */) {
		multirotor_rate_control_params_init();
		initialized = 1 /* true */;

		pid_init(&pitch_rate_controller, multirotor_rate_control_parameters.attrate_p, multirotor_rate_control_parameters.attrate_i,
				multirotor_rate_control_parameters.attrate_d, 1.0f, 1.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.003f);
		pid_init(&roll_rate_controller, multirotor_rate_control_parameters.attrate_p, multirotor_rate_control_parameters.attrate_i,
				multirotor_rate_control_parameters.attrate_d, 1.0f, 1.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.003f);

	}

	/* load new parameters with lower rate */
	if (motor_skip_counter % 2500 == 0) {
		/* update parameters from storage */
		multirotor_rate_control_params_update();
		pid_set_parameters(&pitch_rate_controller, multirotor_rate_control_parameters.attrate_p, multirotor_rate_control_parameters.attrate_i,
				multirotor_rate_control_parameters.attrate_d, 1.0f, 1.0f);
		pid_set_parameters(&roll_rate_controller,  multirotor_rate_control_parameters.attrate_p, multirotor_rate_control_parameters.attrate_i,
				multirotor_rate_control_parameters.attrate_d, 1.0f, 1.0f);
	}

	/* reset integrals if needed */
	if (reset_integral) {
		pid_reset_integral(&pitch_rate_controller);
		pid_reset_integral(&roll_rate_controller);
		// TODO pid_reset_integral(&yaw_rate_controller);
	}

	/* control pitch (forward) output */
	float pitch_control = pid_calculate(&pitch_rate_controller, rate_sp->pitch ,
					    rates[1], 0.0f, deltaT);

	/* control roll (left/right) output */
	float roll_control = pid_calculate(&roll_rate_controller, rate_sp->roll ,
					   rates[0], 0.0f, deltaT);

	/* control yaw rate */ //XXX use library here
	float yaw_rate_control = multirotor_rate_control_parameters.yawrate_p * (rate_sp->yaw - rates[2]);

	/* increase resilience to faulty control inputs */
	if (!check_finite(yaw_rate_control)) {
		yaw_rate_control = 0.0f;
		fprintf (stderr, "WARNING: Attitude controller has computed a NaN yaw control\n");
	}

	actuators->control[0] = roll_control;
	actuators->control[1] = pitch_control;
	actuators->control[2] = yaw_rate_control;
	actuators->control[3] = rate_sp->thrust;

	motor_skip_counter++;
}
