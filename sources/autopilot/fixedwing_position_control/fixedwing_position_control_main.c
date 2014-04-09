/**
 * @file fixedwing_position_control_main.c
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

#include "../../uav_library/common.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/pid/pid.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"

#include "fixedwing_position_control_main.h"
#include "fixedwing_position_control_params.h"



/*
struct planned_path_segments_s {
	bool_t segment_type;
	double start_lat;			// Start of line or center of arc
	double start_lon;
	double end_lat;
	double end_lon;
	float radius;				// Radius of arc
	float arc_start_bearing;		// Bearing from center to start of arc
	float arc_sweep;				// Angle (radians) swept out by arc around center.
	// Positive for clockwise, negative for counter-clockwise
};
*/

static const float M_DEG_TO_RAD = 0.01745329251994;

static absolute_time last_run = 0;
static bool_t global_sp_updated_set_once = 0 /* false */;
static int counter = 0;

static PID_t heading_controller;
static PID_t heading_rate_controller;
static PID_t offtrack_controller;
static PID_t altitude_controller;



/* Main Thread */
void* fixedwing_position_control_thread_main (void *args)
{
	/* welcome user */
	fprintf (stdout, "Fixedwing osition controller started\n");

	int updated, pos_updated;
	absolute_time usec_max_poll_wait_time = 200000;

	/* Setup of loop */
	float deltaT, psi_track = 0.0f;
	float delta_psi_c, psi_c, psi_e;
	float delta_psi_rate_c, psi_rate_track, psi_rate_c;
	float psi_rate_e, ground_speed, psi_rate_e_scaled;

	int distance_res;


	/* declare and safely initialize all structs */
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct vehicle_global_position_s start_pos;		// Temporary variable, replace with
	memset(&start_pos, 0, sizeof(start_pos));		// previous waypoint when available
	struct vehicle_global_position_setpoint_s global_setpoint;
	memset(&global_setpoint, 0, sizeof(global_setpoint));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct crosstrack_error_s xtrack_err;
	memset(&xtrack_err, 0, sizeof(xtrack_err));
	struct parameter_update_s param_update;
	memset(&param_update, 0, sizeof(param_update));


	/* output structs */
	struct vehicle_attitude_setpoint_s attitude_setpoint;
	memset(&attitude_setpoint, 0, sizeof(attitude_setpoint));

	/* publish attitude setpoint */
	attitude_setpoint.roll_body = 0.0f;
	attitude_setpoint.pitch_body = 0.0f;
	attitude_setpoint.yaw_body = 0.0f;
	attitude_setpoint.thrust = 0.0f;

	/* publish the attitude setpoint */
	orb_advert_t attitude_setpoint_adv = orb_advertise(ORB_ID(vehicle_attitude_setpoint));
	if (attitude_setpoint_adv < 0) {
		fprintf (stderr, "Position controller thread failed to advertise vehicle_attitude_setpoint topic\n");
		exit(-1);
	}
	orb_publish(ORB_ID(vehicle_attitude_setpoint), attitude_setpoint_adv, &attitude_setpoint);


	/* subscribe */
	orb_subscr_t global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	if (global_pos_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to vehicle_global_position topic\n");
		exit(-1);
	}

	orb_subscr_t global_setpoint_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	if (global_setpoint_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to vehicle_global_position_setpoint topic\n");
		exit(-1);
	}

	orb_subscr_t att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	if (att_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to vehicle_attitude topic\n");
		exit(-1);
	}

	orb_subscr_t param_sub = orb_subscribe(ORB_ID(parameter_update));
	if (param_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to parameter_update topic\n");
		exit(-1);
	}


	/* abort on a nonzero return value from the parameter init */
	if (fixedwing_position_control_params_init() != 0) {
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Fixedwing position controller aborting on startup due to an error\n");
		exit(-1);
	}

	pid_init(&heading_controller, fixedwing_position_control_parameters.heading_p, 0.0f, 0.0f, 0.0f, 10000.0f, PID_MODE_DERIVATIV_NONE, 0.0f); //arbitrary high limit
	pid_init(&heading_rate_controller, fixedwing_position_control_parameters.headingr_p, fixedwing_position_control_parameters.headingr_i, 0.0f, 0.0f, fixedwing_position_control_parameters.roll_lim, PID_MODE_DERIVATIV_NONE, 0.0f);
	pid_init(&altitude_controller, fixedwing_position_control_parameters.altitude_p, 0.0f, 0.0f, 0.0f, fixedwing_position_control_parameters.pitch_lim, PID_MODE_DERIVATIV_NONE, 0.0f);
	pid_init(&offtrack_controller, fixedwing_position_control_parameters.xtrack_p, 0.0f, 0.0f, 0.0f , 60.0f * M_DEG_TO_RAD, PID_MODE_DERIVATIV_NONE, 0.0f); //TODO: remove hardcoded value


	while (!_shutdown_all_systems) {
		/* wait for a sensor update, check for exit condition every 500 ms */

		/* update parameters */
		updated = orb_check(ORB_ID(parameter_update), param_sub);
		if (updated) {
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(parameter_update), param_sub, &param_update);

			/* update parameters from storage */
			fixedwing_position_control_params_update ();
			pid_set_parameters(&heading_controller, fixedwing_position_control_parameters.heading_p, 0, 0, 0, 10000.0f); //arbitrary high limit
			pid_set_parameters(&heading_rate_controller, fixedwing_position_control_parameters.headingr_p, fixedwing_position_control_parameters.headingr_i, 0, 0, fixedwing_position_control_parameters.roll_lim);
			pid_set_parameters(&altitude_controller, fixedwing_position_control_parameters.altitude_p, 0, 0, 0, fixedwing_position_control_parameters.pitch_lim);
			pid_set_parameters(&offtrack_controller, fixedwing_position_control_parameters.xtrack_p, 0, 0, 0, 60.0f * M_DEG_TO_RAD); //TODO: remove hardcoded value
		}

		/* wait for up to 200ms for data */
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
		/* load local copies */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);


		deltaT = (get_absolute_time() - last_run) / 1000000.0f;
		last_run = get_absolute_time();

		/* check if there is a new position or setpoint */
		pos_updated = orb_check(ORB_ID(vehicle_global_position), global_pos_sub);
		if (pos_updated) {
			orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
		}

		if (orb_check(ORB_ID(vehicle_global_position_setpoint), global_setpoint_sub)) {
			orb_copy(ORB_ID(vehicle_global_position_setpoint), global_setpoint_sub, &global_setpoint);
			start_pos = global_pos; //for now using the current position as the startpoint (= approx. last waypoint because the setpoint switch occurs at the waypoint)
			global_sp_updated_set_once = 1 /* true */;
			psi_track = get_bearing_to_next_waypoint((double)global_pos.latitude, (double)global_pos.longitude,
					(double)global_setpoint.latitude, (double)global_setpoint.longitude);

			fprintf(stdout, "INFO: Next waypoint direction: %0.6f\n", (double)psi_track);
		}

		/* Simple Horizontal Control */
		if (global_sp_updated_set_once) {
			// if (getenv("VERY_VERBOSE") && counter % 100 == 0)
				// printf("lat_sp %d, ln_sp %d, lat: %d, lon: %d\n", global_setpoint.lat, global_setpoint.lon, global_pos.lat, global_pos.lon);

			/* calculate crosstrack error */
			// Only the case of a straight line track following handled so far
			distance_res = get_distance_to_line(&xtrack_err, (double)global_pos.latitude, (double)global_pos.longitude,
								(double)start_pos.latitude, (double)start_pos.longitude,
								(double)global_setpoint.latitude, (double)global_setpoint.longitude);

			if (distance_res == 0 /* OK */) {

				delta_psi_c = pid_calculate(&offtrack_controller, 0, xtrack_err.distance, 0.0f, 0.0f); //fixedwing_position_control_parameters.xtrack_p * xtrack_err.distance
				psi_c = psi_track + delta_psi_c;
				psi_e = psi_c - att.yaw;

				/* wrap difference back onto -pi..pi range */
				psi_e = _wrap_pi(psi_e);

				// if (getenv("VERY_VERBOSE") && counter % 100 == 0)
					// printf("xtrack_err.distance %.4f ", (double)xtrack_err.distance);
					// printf("delta_psi_c %.4f ", (double)delta_psi_c);
					// printf("psi_c %.4f ", (double)psi_c);
					// printf("att.yaw %.4f ", (double)att.yaw);
					// printf("psi_e %.4f ", (double)psi_e);
				// }

				/* calculate roll setpoint, do this artificially around zero */
				delta_psi_rate_c = pid_calculate(&heading_controller, psi_e, 0.0f, 0.0f, 0.0f);
				psi_rate_track = 0; //=V_gr/r_track , this will be needed for implementation of arc following
				psi_rate_c = delta_psi_rate_c + psi_rate_track;

				/* limit turn rate */
				if (psi_rate_c > fixedwing_position_control_parameters.headingr_lim) {
					psi_rate_c = fixedwing_position_control_parameters.headingr_lim;

				} else if (psi_rate_c < -fixedwing_position_control_parameters.headingr_lim) {
					psi_rate_c = -fixedwing_position_control_parameters.headingr_lim;
				}

				psi_rate_e = psi_rate_c - att.yaw_rate;

				// XXX sanity check: Assume 10 m/s stall speed and no stall condition
				ground_speed = sqrtf(global_pos.vx * global_pos.vx + global_pos.vy * global_pos.vy);

				if (ground_speed < 10.0f) {
					ground_speed = 10.0f;
				}

				psi_rate_e_scaled = psi_rate_e * ground_speed / 9.81f; //* V_gr / g

				attitude_setpoint.roll_body = pid_calculate(&heading_rate_controller, psi_rate_e_scaled, 0.0f, 0.0f, deltaT);

				// if (getenv("VERY_VERBOSE") && counter % 100 == 0) {
					// printf("psi_rate_c %.4f ", (double)psi_rate_c);
					// printf("psi_rate_e_scaled %.4f ", (double)psi_rate_e_scaled);
					// printf("rollbody %.4f\n", (double)attitude_setpoint.roll_body);

					// printf("xtrack_err.distance: %0.4f, delta_psi_c: %0.4f\n", xtrack_err.distance, delta_psi_c);
				// }

			} else {
				// if (getenv("VERY_VERBOSE") && counter % 100 == 0)
					// printf("distance_res: %d, past_end %d\n", distance_res, xtrack_err.past_end);
			}

			/* Very simple Altitude Control */
			if (pos_updated) {

				//TODO: take care of relative vs. ab. altitude
				attitude_setpoint.pitch_body = pid_calculate(&altitude_controller, global_setpoint.altitude, global_pos.altitude, 0.0f, 0.0f);

			}

			// XXX need speed control
			attitude_setpoint.thrust = 0.7f;

			/* publish the attitude setpoint */
			orb_publish(ORB_ID(vehicle_attitude_setpoint), attitude_setpoint_adv, &attitude_setpoint);

			counter++;

		} else {
			// XXX no setpoint, decent default needed (loiter?)
		}
	}

	/*
	 * do unsubscriptions
	 */
	orb_unsubscribe(ORB_ID(vehicle_global_position_setpoint), global_setpoint_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_global_position), global_pos_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude), att_sub, pthread_self());
	orb_unsubscribe(ORB_ID(parameter_update), param_sub, pthread_self());

	/*
	 * do unadvertises
	 */
	orb_unadvertise(ORB_ID(vehicle_attitude_setpoint), attitude_setpoint_adv, pthread_self());

	return 0;

}
