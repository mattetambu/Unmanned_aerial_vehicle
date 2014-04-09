/**
 * @file fixedwing_attitude_control_rate.c
 *
 * Implementation of a fixed wing attitude controller.
 *
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

#include "fixedwing_attitude_control_rate.h"
#include "fixedwing_attitude_control_rate_params.h"



static int counter = 0;
static bool_t initialized = 0;
static absolute_time last_run = 0;

static PID_t roll_rate_controller;
static PID_t pitch_rate_controller;
static PID_t yaw_rate_controller;


int fixedwing_attitude_control_rates(const struct vehicle_rates_setpoint_s *rate_sp, const float rates[], struct actuator_controls_s *actuators)
{
	float deltaT = (get_absolute_time() - last_run) / 1000000.0f;
	last_run = get_absolute_time();

	if (!initialized) {
		fixedwing_attitude_control_rate_params_init();
		pid_init(&roll_rate_controller, fixedwing_attitude_control_rate_parameters.rollrate_p,
				fixedwing_attitude_control_rate_parameters.rollrate_i, 0,
				fixedwing_attitude_control_rate_parameters.rollrate_awu, 1, PID_MODE_DERIVATIV_NONE, 0.0f); // set D part to 0 because the controller layout is with a PI rate controller
		pid_init(&pitch_rate_controller, fixedwing_attitude_control_rate_parameters.pitchrate_p,
				fixedwing_attitude_control_rate_parameters.pitchrate_i, 0,
				fixedwing_attitude_control_rate_parameters.pitchrate_awu, 1, PID_MODE_DERIVATIV_NONE, 0.0f); // set D part to 0 because the contpitcher layout is with a PI rate contpitcher
		pid_init(&yaw_rate_controller, fixedwing_attitude_control_rate_parameters.yawrate_p,
				fixedwing_attitude_control_rate_parameters.yawrate_i, 0,
				fixedwing_attitude_control_rate_parameters.yawrate_awu, 1, PID_MODE_DERIVATIV_NONE, 0.0f); // set D part to 0 because the contpitcher layout is with a PI rate contpitcher

		initialized = 1;
	}

	/* load new parameters with lower rate */
	if (counter % 100 == 0) {
		/* update parameters from storage */
		fixedwing_attitude_control_rate_params_update();
		pid_set_parameters(&roll_rate_controller, fixedwing_attitude_control_rate_parameters.rollrate_p,
				fixedwing_attitude_control_rate_parameters.rollrate_i, 0, fixedwing_attitude_control_rate_parameters.rollrate_awu, 1);
		pid_set_parameters(&pitch_rate_controller, fixedwing_attitude_control_rate_parameters.pitchrate_p,
				fixedwing_attitude_control_rate_parameters.pitchrate_i, 0, fixedwing_attitude_control_rate_parameters.pitchrate_awu, 1);
		pid_set_parameters(&yaw_rate_controller, fixedwing_attitude_control_rate_parameters.yawrate_p,
				fixedwing_attitude_control_rate_parameters.yawrate_i, 0, fixedwing_attitude_control_rate_parameters.yawrate_awu, 1);
	}


	/* roll rate (PI) */
	actuators->control[0] = pid_calculate(&roll_rate_controller, rate_sp->roll, rates[0], 0.0f, deltaT);
	/* pitch rate (PI) */
	actuators->control[1] = -pid_calculate(&pitch_rate_controller, rate_sp->pitch, rates[1], 0.0f, deltaT);
	/* yaw rate (PI) */
	actuators->control[2] = pid_calculate(&yaw_rate_controller, rate_sp->yaw, rates[2], 0.0f, deltaT);

	counter++;

	return 0;
}



