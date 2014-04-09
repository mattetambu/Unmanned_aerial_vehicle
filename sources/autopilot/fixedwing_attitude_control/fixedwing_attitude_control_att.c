/**
 * @file fixedwing_attitude_control_att.c
 *
 * Implementation of a fixed wing attitude controller.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>

#include "../../ORB/ORB.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"

#include "../../uav_library/common.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/pid/pid.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"

#include "fixedwing_attitude_control_att.h"
#include "fixedwing_attitude_control_att_params.h"


static int counter = 0;
static bool_t initialized = 0;

static PID_t roll_controller;
static PID_t pitch_controller;



int fixedwing_attitude_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
				   const float speed_body[], struct vehicle_rates_setpoint_s *rates_sp)
{
	float pitch_sp_rollcompensation;

	if (!initialized) {
		fixedwing_attitude_control_att_params_init();
		pid_init(&roll_controller, fixedwing_attitude_control_att_parameters.roll_p,
				0, 0, 0, fixedwing_attitude_control_att_parameters.rollrate_lim, PID_MODE_DERIVATIV_NONE, 0.0f); //P Controller
		pid_init(&pitch_controller, fixedwing_attitude_control_att_parameters.pitch_p,
				0, 0, 0, fixedwing_attitude_control_att_parameters.pitchrate_lim, PID_MODE_DERIVATIV_NONE, 0.0f); //P Controller

		initialized = 1;
	}

	/* load new parameters with lower rate */
	if (counter % 100 == 0) {
		/* update parameters from storage */
		fixedwing_attitude_control_att_params_update();
		pid_set_parameters(&roll_controller, fixedwing_attitude_control_att_parameters.roll_p,
				0, 0, 0, fixedwing_attitude_control_att_parameters.rollrate_lim);
		pid_set_parameters(&pitch_controller, fixedwing_attitude_control_att_parameters.pitch_p,
				0, 0, 0, fixedwing_attitude_control_att_parameters.pitchrate_lim);
	}

	/* Roll (P) */
	rates_sp->roll = pid_calculate(&roll_controller, att_sp->roll_body, att->roll, 0, 0);


	/* Pitch (P) */
	/* compensate feedforward for loss of lift due to non-horizontal angle of wing */
	pitch_sp_rollcompensation = fixedwing_attitude_control_att_parameters.pitch_roll_compensation_p * fabsf(sinf(att_sp->roll_body));
	/* set pitch plus feedforward roll compensation */
	rates_sp->pitch = pid_calculate(&pitch_controller, att_sp->pitch_body + pitch_sp_rollcompensation, att->pitch, 0, 0);

	/* Yaw (from coordinated turn constraint or lateral force) */
	rates_sp->yaw = (att->roll_rate * rates_sp->roll + 9.81f * sinf(att->roll) * cosf(att->pitch) + speed_body[0] * rates_sp->pitch * sinf(att->roll))
					/ (speed_body[0] * cosf(att->roll) * cosf(att->pitch) + speed_body[2] * sinf(att->pitch));

	// printf("rates_sp->yaw %.4f \n", (double)rates_sp->yaw);

	counter++;

	return 0;
}



