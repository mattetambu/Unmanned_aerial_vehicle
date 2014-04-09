/**
 * @file fixedwing_attitude_control.c
 * Implementation of a fixed wing attitude controller.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <time.h>

#include "../../ORB/ORB.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/parameter_update.h"
#include "../../ORB/topics/position/vehicle_global_position.h"
#include "../../ORB/topics/setpoint/vehicle_global_position_setpoint.h"
#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
#include "../../ORB/topics/vehicle_status.h"
#include "../../ORB/topics/vehicle_control_flags.h"

#include "../../uav_library/common.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/pid/pid.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/math/limits.h"

#include "fixedwing_attitude_control_main.h"
#include "fixedwing_attitude_control_rate.h"
#include "fixedwing_attitude_control_att.h"
#include "fixedwing_attitude_control_rate_params.h"
#include "fixedwing_attitude_control_att_params.h"


/* Main Thread */
void* fixedwing_attitude_control_thread_main (void *args)
{
	/* welcome user */
	fprintf (stdout, "Fixedwing attitude controller started\n");

	int i, updated;
	absolute_time usec_max_poll_wait_time = 500000;

	/* Setup of loop */
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float speed_body[3] = {0.0f, 0.0f, 0.0f};


	/* declare and safely initialize all structs */
	struct vehicle_attitude_s att;
	struct vehicle_attitude_setpoint_s att_sp;
	struct vehicle_global_position_s global_pos;
	struct vehicle_rates_setpoint_s rates_sp;
	struct manual_control_setpoint_s manual_sp;
	struct vehicle_control_flags_s control_flags;
	struct vehicle_status_s vstatus;
	memset(&att, 0, sizeof(att));
	memset(&att_sp, 0, sizeof(att_sp));
	memset(&rates_sp, 0, sizeof(rates_sp));
	memset(&global_pos, 0, sizeof(global_pos));
	memset(&manual_sp, 0, sizeof(manual_sp));
	memset(&control_flags, 0, sizeof(control_flags));
	memset(&vstatus, 0, sizeof(vstatus));

	/* output structs */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	/* abort on a nonzero return value from the parameter init */
	if (fixedwing_attitude_control_rate_params_init() != 0 ||
		fixedwing_attitude_control_att_params_init() != 0) {
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Fixedwing attitude controller aborting on startup due to an error\n");
		exit(-1);
	}


	/* advertise */
	orb_advert_t actuator_adv = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	if (actuator_adv < 0) {
		fprintf (stderr, "Attitude controller thread failed to advertise actuator_controls topic\n");
		exit(-1);
	}
	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_adv, &actuators);

	orb_advert_t rates_adv = orb_advertise(ORB_ID(vehicle_rates_setpoint));
	if (rates_adv < 0) {
		fprintf (stderr, "Attitude controller thread failed to advertise vehicle_attitude_setpoint topic\n");
		exit(-1);
	}
	orb_publish(ORB_ID(vehicle_rates_setpoint), rates_adv, &rates_sp);


	/* subscribe */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	if (att_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_attitude topic\n");
		exit(-1);
	}

	int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	if (att_sp_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_attitude_setpoint topic\n");
		exit(-1);
	}

	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	if (global_pos_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_global_position topic\n");
		exit(-1);
	}

	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	if (manual_sp_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to manual_control_setpoint topic\n");
		exit(-1);
	}

	int control_flags_sub = orb_subscribe(ORB_ID(vehicle_control_flags));
	if (control_flags_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_control_flags topic\n");
		exit(-1);
	}

	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	if (vehicle_status_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_status topic\n");
		exit(-1);
	}


	while (!_shutdown_all_systems) {

		/* wait for up to 500ms for data */
		updated = orb_poll(ORB_ID(vehicle_attitude), att_sub, usec_max_poll_wait_time);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (!updated)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (updated < 0) {
			fprintf (stderr, "Attitude controller failed to poll vehicle attitude\n");
			continue;
		}


		/* only run controller if attitude changed */
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

		/* Check if there is a new position measurement or  attitude setpoint */
		if (orb_check(ORB_ID(vehicle_attitude_setpoint), att_sp_sub))
			orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);

		if (orb_check(ORB_ID(vehicle_global_position), global_pos_sub)) {
			orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);

			if (att.R_valid) {
				speed_body[0] = att.R[0][0] * global_pos.vx + att.R[0][1] * global_pos.vy + att.R[0][2] * global_pos.vz;
				speed_body[1] = att.R[1][0] * global_pos.vx + att.R[1][1] * global_pos.vy + att.R[1][2] * global_pos.vz;
				speed_body[2] = att.R[2][0] * global_pos.vx + att.R[2][1] * global_pos.vy + att.R[2][2] * global_pos.vz;

			} else {
				speed_body[0] = 0;
				speed_body[1] = 0;
				speed_body[2] = 0;

				fprintf (stderr, "WARNING: Attitude control did not get a valid rotation matrix\n");
			}
		}

		orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
		orb_copy(ORB_ID(vehicle_control_flags), control_flags_sub, &control_flags);
		orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

		gyro[0] = att.roll_rate;
		gyro[1] = att.pitch_rate;
		gyro[2] = att.yaw_rate;

		/* set manual setpoints if required */
		if (control_flags.flag_control_manual_enabled) {
			if (control_flags.flag_control_attitude_enabled) {

				/* if the RC signal is lost, try to stay level and go slowly back down to ground */
				if (vstatus.rc_signal_lost) {

					/* put plane into loiter */
					att_sp.roll_body = 0.3f;
					att_sp.pitch_body = 0.0f;

					/* limit throttle to 60 % of last value if sane */
					if (check_finite(manual_sp.thrust) &&
					    (manual_sp.thrust >= 0.0f) &&
					    (manual_sp.thrust <= 1.0f)) {
						att_sp.thrust = 0.6f * manual_sp.thrust;

					} else {
						att_sp.thrust = 0.0f;
					}

					att_sp.yaw_body = 0;

					// XXX disable yaw control, loiter

				} else {

					att_sp.roll_body = manual_sp.roll;
					att_sp.pitch_body = manual_sp.pitch;
					att_sp.yaw_body = 0;
					att_sp.thrust = manual_sp.thrust;
				}

				att_sp.timestamp = get_absolute_time();

				/* pass through flaps */
				if (check_finite(manual_sp.flaps)) {
					actuators.control[4] = manual_sp.flaps;

				} else {
					actuators.control[4] = 0.0f;
				}

			} else {
				/* directly pass through values */
				actuators.control[0] = manual_sp.roll;
				/* positive pitch means negative actuator -> pull up */
				actuators.control[1] = manual_sp.pitch;
				actuators.control[2] = manual_sp.yaw;
				actuators.control[3] = manual_sp.thrust;

				if (check_finite(manual_sp.flaps)) {
					actuators.control[4] = manual_sp.flaps;

				} else {
					actuators.control[4] = 0.0f;
				}
			}
		}
		
		/* execute attitude control if requested */
		if (control_flags.flag_control_attitude_enabled) {
			/* attitude control */
			fixedwing_attitude_control_attitude(&att_sp, &att, speed_body, &rates_sp);

			/* angular rate control */
			fixedwing_attitude_control_rates(&rates_sp, gyro, &actuators);

			/* pass through throttle */
			actuators.control[3] = att_sp.thrust;

			/* set flaps to zero */
			actuators.control[4] = 0.0f;

		}

		/* publish rates */
		orb_publish(ORB_ID(vehicle_rates_setpoint), rates_adv, &rates_sp);

		/* sanity check and publish actuator outputs */
		if (check_finite(actuators.control[0]) &&
		    check_finite(actuators.control[1]) &&
		    check_finite(actuators.control[2]) &&
		    check_finite(actuators.control[3])) {
			orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_adv, &actuators);
		}
	}


	fprintf (stdout, "INFO: Attitude controller exiting, disarming motors\n");
	fflush (stdout);

	/* kill all outputs */
	for (i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_adv, &actuators);


	/*
	 * do unsubscriptions
	 */
	orb_unsubscribe(ORB_ID(vehicle_attitude), att_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_global_position), global_pos_sub, pthread_self());
	orb_unsubscribe(ORB_ID(manual_control_setpoint), manual_sp_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_control_flags), control_flags_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_status), vehicle_status_sub, pthread_self());

	/*
	 * do unadvertises
	 */
	orb_unadvertise(ORB_ID(vehicle_rates_setpoint), rates_adv, pthread_self());
	orb_unadvertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_adv, pthread_self());

	return 0;
}
