/*
 * @file multirotor_attitude_control.c
 *
 * Implementation of attitude controller for multirotors.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "../../uav_library/common.h"
#include "../../uav_library/pid/pid.h"
#include "../../uav_library/time/drv_time.h"
#include "../../ORB/ORB.h"

#include "multirotor_attitude_control.h"
#include "multirotor_attitude_control_params.h"


static absolute_time last_run = 0;
static absolute_time last_input = 0;
static int motor_skip_counter = 0;
static bool_t initialized = 0 /* false */;

static PID_t pitch_controller;
static PID_t roll_controller;


void multirotor_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp,
				 const struct vehicle_attitude_s *att, struct vehicle_rates_setpoint_s *rates_sp, bool_t control_yaw_position, bool_t reset_integral)
{
	float yaw_error;
	float deltaT = (get_absolute_time() - last_run) / 1000000.0f;
	last_run = get_absolute_time();


	if (last_input != att_sp->timestamp) {
		last_input = att_sp->timestamp;
	}

	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == 0 /* false */) {
		multirotor_attitude_control_params_init ();

		pid_init(&pitch_controller, multirotor_attitude_control_parameters.att_p, multirotor_attitude_control_parameters.att_i, multirotor_attitude_control_parameters.att_d, 1000.0f, 1000.0f, PID_MODE_DERIVATIV_SET, 0.0f);
		pid_init(&roll_controller, multirotor_attitude_control_parameters.att_p, multirotor_attitude_control_parameters.att_i, multirotor_attitude_control_parameters.att_d, 1000.0f, 1000.0f, PID_MODE_DERIVATIV_SET, 0.0f);

		initialized = 1 /* true */;
	}

	/* load new parameters with lower rate */
	if (motor_skip_counter % 500 == 0) {
		/* update parameters from storage */
		multirotor_attitude_control_params_update ();

		/* apply parameters */
		pid_set_parameters(&pitch_controller, multirotor_attitude_control_parameters.att_p, multirotor_attitude_control_parameters.att_i, multirotor_attitude_control_parameters.att_d, 1000.0f, 1000.0f);
		pid_set_parameters(&roll_controller, multirotor_attitude_control_parameters.att_p, multirotor_attitude_control_parameters.att_i, multirotor_attitude_control_parameters.att_d, 1000.0f, 1000.0f);
	}

	/* reset integrals if needed */
	if (reset_integral) {
		pid_reset_integral(&pitch_controller);
		pid_reset_integral(&roll_controller);
		//TODO pid_reset_integral(&yaw_controller);
	}

	/* calculate current control outputs */

	/* control pitch (forward) output */
	rates_sp->pitch = pid_calculate(&pitch_controller, att_sp->pitch_body ,
					att->pitch, att->pitch_rate, deltaT);

	/* control roll (left/right) output */
	rates_sp->roll = pid_calculate(&roll_controller, att_sp->roll_body ,
				       att->roll, att->roll_rate, deltaT);

	if (control_yaw_position) {
		/* control yaw rate */
		// TODO use pid lib

		/* positive error: rotate to right, negative error, rotate to left (NED frame) */
		// yaw_error = _wrap_pi(att_sp->yaw_body - att->yaw);

		yaw_error = att_sp->yaw_body - att->yaw;

		if (yaw_error > M_PI) {
			yaw_error -= M_TWOPI;

		} else if (yaw_error < -M_PI) {
			yaw_error += M_TWOPI;
		}

		rates_sp->yaw = multirotor_attitude_control_parameters.yaw_p * (yaw_error) - (multirotor_attitude_control_parameters.yaw_d * att->yaw_rate);
	}

	rates_sp->thrust = att_sp->thrust;
    //need to update the timestamp now that we've touched rates_sp
    rates_sp->timestamp = get_absolute_time();

	motor_skip_counter++;
}
