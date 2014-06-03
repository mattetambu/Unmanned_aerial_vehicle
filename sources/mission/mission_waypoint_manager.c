/**
 * @file mission_waypoint_manager.c
 * Mission waypoint manager
 */

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>

#include "../uav_library/common.h"
#include "../uav_library/param/param.h"
#include "../uav_library/time/drv_time.h"
#include "../uav_library/math/limits.h"
#include "../uav_library/geo/geo.h"
#include "../uav_type.h"

#include "../ORB/ORB.h"
#include "../ORB/topics/mission.h"
#include "../ORB/topics/parameter_update.h"
#include "../ORB/topics/navigation_capabilities.h"
#include "../ORB/topics/vehicle_control_flags.h"
#include "../ORB/topics/position/vehicle_global_position.h"
#include "../ORB/topics/position/home_position.h"
#include "../ORB/topics/setpoint/vehicle_global_position_setpoint.h"
#include "../ORB/topics/setpoint/vehicle_global_position_set_triplet.h"

#include "mission_waypoint_manager.h"
#include "mission_waypoint_manager_params.h"


#define MISSION_WPM_TEXT_FEEDBACK	1

// function prototypes
void missionlib_current_waypoint_changed ();
void find_next_setpoint_index (int curr_index, int *ret_index, bool_t *ret_valid);


/*
 * Advertise
 */
static orb_advert_t global_position_setpoint_pub = -1;
static orb_advert_t global_position_set_triplet_pub = -1;

struct vehicle_global_position_setpoint_s g_sp;	/* global, absolute waypoint */
struct vehicle_global_position_set_triplet_s triplet;	/* fill triplet: previous, current, next waypoint */

/*
 * Waypoint manager
 */
static mission_t mission;
static home_position_s home_position;


struct {
	int last_setpoint_index;
	int current_setpoint_index;
	int next_setpoint_index;

	bool_t last_setpoint_valid;
	bool_t current_setpoint_valid;
	bool_t next_setpoint_valid;

	absolute_time loiter_start_time;
	absolute_time timestamp_firstinside_orbit;
	absolute_time timestamp_lastoutside_orbit;
	bool_t yaw_reached;
	bool_t pos_reached;

	bool_t initialized;
} wp_manager;




int wp_manager_init (orb_subscr_t mission_sub, orb_subscr_t home_sub)
{
	float safe_loiter_altitude = mission_waypoint_manager_parameters.safe_loiter_altitude;
	float loiter_radius = mission_waypoint_manager_parameters.loiter_radius;

	// Set all waypoints to zero
	wp_manager.last_setpoint_index = 0;
	wp_manager.current_setpoint_index = 0;
	wp_manager.next_setpoint_index = 0;

	wp_manager.last_setpoint_valid = 0;
	wp_manager.current_setpoint_valid = 0;
	wp_manager.next_setpoint_valid = 0;

	wp_manager.yaw_reached = 0;						///< boolean for yaw attitude reached
	wp_manager.pos_reached = 0;						///< boolean for position reached

	wp_manager.loiter_start_time = 0;
	wp_manager.timestamp_lastoutside_orbit = 0;		///< timestamp when the MAV was last outside the orbit or had the wrong yaw value
	wp_manager.timestamp_firstinside_orbit = 0;		///< timestamp when the MAV was the first time after a waypoint change inside the orbit and had the correct yaw value

	if (orb_wait (ORB_ID(mission), mission_sub) > 0)
		orb_copy (ORB_ID(mission), mission_sub, (void *) &mission);
	else
	{
		fprintf (stderr, "Waypoint manager thread experienced an error waiting for mission topic\n");
		return -1;
	}

	find_next_setpoint_index (-1, &wp_manager.current_setpoint_index, &wp_manager.current_setpoint_valid);

	if (mission.n_commands == 0 || !wp_manager.current_setpoint_valid)
	{
		fprintf (stderr, "Waypoint manager can't use a mission with no valid commands\n");
		return -1;
	}

	if (orb_wait (ORB_ID(home_position), home_sub) > 0)
		orb_copy (ORB_ID(home_position), home_sub, (void *) &home_position);
	else
	{
		fprintf (stderr, "Waypoint manager thread experienced an error waiting for home_position topic\n");
		return -1;
	}

	/*
	 * if the last waypoint is NOT a land waypoint then let the system return to home and loitering there.
	 */
	if (mission.command_list[mission.n_commands - 1].name != accepted_command_land) {
		mission.command_list[mission.n_commands].name = accepted_command_loiter_unlim;
		mission.command_list[mission.n_commands].option1 = home_position.altitude + safe_loiter_altitude;
		mission.command_list[mission.n_commands].option2 = home_position.latitude;
		mission.command_list[mission.n_commands].option3 = home_position.longitude;
		mission.command_list[mission.n_commands].option4 = loiter_radius;
		mission.command_list[mission.n_commands].option5 = 0; // means clockwise

		mission.n_commands++;
	}

	mission.command_list[wp_manager.current_setpoint_index].current = 1;
	find_next_setpoint_index (wp_manager.current_setpoint_index, &wp_manager.next_setpoint_index, &wp_manager.next_setpoint_valid);

	wp_manager.initialized = 1;
	missionlib_current_waypoint_changed ();

	return 0;
}

/*
 * Calculate distance in global frame.
 *
 * The distance calculation is based on the WGS84 geoid (GPS)
 */
float wpm_distance_to_point_global_wgs84(float sp_alt, float sp_lat, float sp_lon, float alt, float lat, float lon, float *dist_xy, float *dist_z)
{
	float dxy = get_distance_to_next_waypoint (lat, lon, sp_lat, sp_lon);
	float dz = alt - sp_alt;

	*dist_xy = fabsf(dxy);
	*dist_z = fabsf(dz);

	return sqrtf(dxy * dxy + dz * dz);
}


int get_yaw_to_next_waypoint (int curr_index, int next_index)
{
	mission_command_t curr_cmd = mission.command_list[curr_index];
	mission_command_t next_cmd = mission.command_list[next_index];
	float curr_lat, curr_lon, next_lat, next_lon;

	curr_lat = (curr_cmd.name == accepted_command_rtl ||
				curr_cmd.name == accepted_command_land)? home_position.latitude : curr_cmd.option2;
	curr_lon = (curr_cmd.name == accepted_command_rtl ||
				curr_cmd.name == accepted_command_land)? home_position.longitude : curr_cmd.option3;

	next_lat = (next_cmd.name == accepted_command_rtl ||
				next_cmd.name == accepted_command_land)? home_position.latitude : next_cmd.option2;
	next_lon = (next_cmd.name == accepted_command_rtl ||
				next_cmd.name == accepted_command_land)? home_position.longitude : next_cmd.option3;

	return get_bearing_to_next_waypoint(curr_lat, curr_lon, next_lat, next_lon);
}


void find_next_setpoint_index (int curr_index, int *ret_index, bool_t *ret_valid)
{
	int next_index = 0, next_valid = 0;
	mission_command_t *next_command;

	/* next waypoint */
	if (mission.n_commands > 0) {
		next_index = curr_index + 1;
	}

	while (next_index < mission.n_commands) {
		next_command = &mission.command_list[next_index];

		if (next_command->name == accepted_command_waypoint ||
			next_command->name == accepted_command_loiter_time ||
			next_command->name == accepted_command_loiter_circle ||
			next_command->name == accepted_command_loiter_unlim ||
			next_command->name == accepted_command_rtl ||
			next_command->name == accepted_command_takeoff ||	// XXX check this
			next_command->name == accepted_command_land) {
			next_valid = 1;
			break;
		}

		if (next_command->name == accepted_command_end_repeat)
		{
			// update repeat counter
			// XXX maybe someone want to know that the counter is updated (publish?)
			next_command->option1++;
			mission.command_list[(int)next_command->option4].option1++;

			if (next_command->option1 < next_command->option3)
				next_index = next_command->option4;
		}

		next_index++;
	}

	*ret_valid = next_valid;
	if (next_valid)
		*ret_index = next_index;
}



/**
 * Set special vehicle setpoint fields based on current mission item.
 *
 * @return 0 if the mission item could be interpreted
 * successfully, and -1 on failure.
 */
int set_special_fields(accepted_command_t command, float option4, float option5, struct vehicle_global_position_setpoint_s *sp)
{
	/* define the turn distance */
	float orbit = mission_waypoint_manager_parameters.orbit_def_radius;

	switch (command) {
		case accepted_command_loiter_unlim:
			sp->nav_cmd = navigation_command_loiter_unlim;
			break;
		case accepted_command_loiter_time:
			sp->nav_cmd = navigation_command_loiter_time;
			wp_manager.loiter_start_time = get_absolute_time();
			break;
		case accepted_command_loiter_circle:
		 	sp->nav_cmd = navigation_command_loiter_circle;
		 	break;
		case accepted_command_waypoint:
			sp->nav_cmd = navigation_command_waypoint;
			break;
		case accepted_command_rtl:
			sp->nav_cmd = navigation_command_rtl;
			break;
		case accepted_command_land:
			sp->nav_cmd = navigation_command_land;
			break;
		case accepted_command_takeoff:
			sp->nav_cmd = navigation_command_takeoff;
			break;
		default:
			/* abort */
			return -1;
	}


	if (command == accepted_command_waypoint) {

		if (option4 >= mission_waypoint_manager_parameters.orbit_min_radius)
			orbit = option4;

	} else if (command == accepted_command_loiter_circle ||
			command == accepted_command_loiter_time ||
			command == accepted_command_loiter_unlim) {

		if (option4 >= mission_waypoint_manager_parameters.orbit_min_radius)
			orbit = option4;
		sp->loiter_radius = orbit;
		sp->loiter_direction = (option5 < 0) ? -1 : 1;

	}

	sp->turn_distance_xy = orbit;
	sp->turn_distance_z = orbit;

	return 0;
}


int set_vehicle_global_position_setpoint (int index, float yaw_sp, struct vehicle_global_position_setpoint_s* g_sp)
{
	mission_command_t cmd = mission.command_list[index];
	float navigation_alt, navigation_lat, navigation_lon;

	navigation_alt = cmd.option1;
	navigation_lat = (cmd.name == accepted_command_rtl ||
					  cmd.name == accepted_command_land)? home_position.latitude : cmd.option2;
	navigation_lon = (cmd.name == accepted_command_rtl ||
					  cmd.name == accepted_command_land)? home_position.longitude : cmd.option3;


	/* Update setpoints */
	g_sp->altitude = navigation_alt;
	g_sp->latitude = navigation_lat * 1e7f;
	g_sp->longitude = navigation_lon * 1e7f;
	g_sp->altitude_is_relative = 0;
	g_sp->yaw = _wrap_pi(yaw_sp / 180.0f * M_PI);

	return set_special_fields(cmd.name, cmd.option4, cmd.option5, g_sp);
}



/*
 *  @brief Directs the MAV to fly to a position
 *
 *  Sends a message to the controller, advising it to fly to the coordinates
 *  of the waypoint with a given orientation
 *  It publishes the vehicle_global_position_setpoint_s
 *
 */
void missionlib_current_waypoint_changed ()
{
	float yaw_sp;

	/* populate last and next */
	triplet.previous_valid = wp_manager.last_setpoint_valid;
	triplet.current_valid = wp_manager.current_setpoint_valid;
	triplet.next_valid = wp_manager.next_setpoint_valid;

	if (triplet.previous_valid) {
		yaw_sp = get_yaw_to_next_waypoint (wp_manager.last_setpoint_index, wp_manager.current_setpoint_index);

		set_vehicle_global_position_setpoint (wp_manager.last_setpoint_index, yaw_sp, &g_sp);
		memcpy(&(triplet.previous), &g_sp, sizeof(g_sp));
	}

	if (triplet.next_valid) {
		yaw_sp = 0; // no need to calculate this for now

		set_vehicle_global_position_setpoint (wp_manager.next_setpoint_index, yaw_sp, &g_sp);
		memcpy(&(triplet.next), &g_sp, sizeof(g_sp));
	}

	if (triplet.current_valid) {
		yaw_sp = get_yaw_to_next_waypoint (wp_manager.current_setpoint_index, wp_manager.next_setpoint_index);

		set_vehicle_global_position_setpoint (wp_manager.current_setpoint_index, yaw_sp, &g_sp);
		memcpy(&(triplet.current), &g_sp, sizeof(g_sp));
	}

	if (MISSION_WPM_TEXT_FEEDBACK)
		printf("INFO: Set new waypoint (%u)\n", wp_manager.current_setpoint_index);

	orb_publish(ORB_ID(vehicle_global_position_setpoint), global_position_setpoint_pub, &g_sp);
	orb_publish(ORB_ID(vehicle_global_position_set_triplet), global_position_set_triplet_pub, &triplet);
}




void check_waypoints_reached(absolute_time now, const struct vehicle_global_position_s *global_pos, float turn_distance)
{
	mission_command_t current_command;
	bool_t time_elapsed = 0;

	float vertical_switch_distance, yaw_sp, yaw_err;
	float dist = -1.0f, dist_xy = -1.0f, dist_z = -1.0f;
	float current_lat = -1.0f, current_lon = -1.0f, current_alt = -1.0f;
	float orbit = mission_waypoint_manager_parameters.orbit_def_radius;
	float orbit_hold_time = mission_waypoint_manager_parameters.orbit_hold_time;

	/* position not valid */
	if (!global_pos->valid || check_out_of_bounds(wp_manager.current_setpoint_index, 0, mission.n_commands)) {

		/* nothing to check here, return */
		return;
	}

	current_command = mission.command_list[wp_manager.current_setpoint_index];

	if (wp_manager.current_setpoint_index < mission.n_commands) {

		if (current_command.name == accepted_command_waypoint ||
			current_command.name == accepted_command_loiter_time ||
			current_command.name == accepted_command_loiter_circle ||
			current_command.name == accepted_command_loiter_unlim) {

			if (current_command.option4 >= mission_waypoint_manager_parameters.orbit_min_radius)
				orbit = current_command.option4;
		}

		/* keep vertical orbit */
		vertical_switch_distance = orbit;

		/* Take the larger turn distance - orbit or turn_distance */
		if (orbit < turn_distance)
			orbit = turn_distance;

		current_alt = current_command.option1;
		current_lat = (current_command.name == accepted_command_rtl ||
					   current_command.name == accepted_command_land)? home_position.latitude : current_command.option2;
		current_lon = (current_command.name == accepted_command_rtl ||
					   current_command.name == accepted_command_land)? home_position.longitude : current_command.option3;

		dist = wpm_distance_to_point_global_wgs84(current_alt, current_lat, current_lon, global_pos->altitude, (float)global_pos->latitude * 1e-7f, (float)global_pos->longitude * 1e-7f, &dist_xy, &dist_z);
		if (dist >= 0.f && dist_xy <= orbit && dist_z >= 0.0f && dist_z <= vertical_switch_distance) {
			wp_manager.pos_reached = 1;
		}

		// check if required yaw reached
		yaw_sp = get_yaw_to_next_waypoint (wp_manager.current_setpoint_index, wp_manager.next_setpoint_index);
		yaw_err = _wrap_pi(yaw_sp - global_pos->yaw);
		if (fabsf(yaw_err) < 0.05f) {
			wp_manager.yaw_reached = 1;
		}
	}

	//check if the current waypoint was reached
	if (wp_manager.pos_reached /* && wp_manager.yaw_reached */) {

		if (wp_manager.timestamp_firstinside_orbit == 0) {
			// Announce that last waypoint was reached
			//wpm_send_waypoint_reached(wp_manager.current_setpoint_index);
			wp_manager.timestamp_firstinside_orbit = now;
		}

		// check if the MAV was long enough inside the waypoint orbit
		if (current_command.name != accepted_command_loiter_unlim &&	// XXX check this
			(now - wp_manager.timestamp_firstinside_orbit >= orbit_hold_time * 1000000 ||
			 current_command.name == accepted_command_takeoff)) {
			time_elapsed = 1;
		}

		if (time_elapsed) {
			current_command.current = 0;
			wp_manager.last_setpoint_index = wp_manager.current_setpoint_index;
			wp_manager.last_setpoint_valid = wp_manager.current_setpoint_valid;

			wp_manager.current_setpoint_index = wp_manager.next_setpoint_index;
			wp_manager.current_setpoint_valid = wp_manager.next_setpoint_valid;

			if (wp_manager.current_setpoint_valid)
				find_next_setpoint_index (wp_manager.current_setpoint_index, &wp_manager.next_setpoint_index, &wp_manager.next_setpoint_valid);

			/* if an error occur abort and return to home (then loiter unlim) */
			if (!wp_manager.current_setpoint_valid) {
				wp_manager.current_setpoint_index = mission.n_commands;
				wp_manager.current_setpoint_valid = 1;
			}


			// Fly to next waypoint
			wp_manager.timestamp_firstinside_orbit = 0;
			wp_manager.timestamp_lastoutside_orbit = 0;
			wp_manager.pos_reached = 0;
			wp_manager.yaw_reached = 0;
			mission.command_list[wp_manager.current_setpoint_index].current = 1;

			//wpm_send_waypoint_current(wp_manager.current_setpoint_index);
			missionlib_current_waypoint_changed();
		}
	}
	else {
		wp_manager.timestamp_lastoutside_orbit = now;
	}
}



/**
 * Mission waypoint manager  main function
 */
void *mission_waypoint_manager_thread_main(void* args)
{
	absolute_time now;
	int updated;
	float turn_distance;

	/* initialize waypoint manager */
	// ************************************* subscribe ******************************************
	global_position_setpoint_pub = orb_advertise (ORB_ID(vehicle_global_position_setpoint));
	if (global_position_setpoint_pub == -1)
	{
		fprintf (stderr, "Waypoint manager thread failed to advertise the vehicle_global_position_setpoint topic\n");
		exit (-1);
	}

	global_position_set_triplet_pub = orb_advertise (ORB_ID(vehicle_global_position_set_triplet));
	if (global_position_set_triplet_pub == -1)
	{
		fprintf (stderr, "Waypoint manager thread failed to advertise the vehicle_global_position_set_triplet topic\n");
		exit (-1);
	}

	// ************************************* subscribe ******************************************
	struct vehicle_global_position_s global_pos;
	orb_subscr_t global_pos_sub = orb_subscribe (ORB_ID(vehicle_global_position));
	if (global_pos_sub == -1)
	{
		fprintf (stderr, "Waypoint manager thread failed to subscribe the vehicle_global_position topic\n");
		exit (-1);
	}

	struct vehicle_control_flags_s control_flags;
	orb_subscr_t control_flags_sub = orb_subscribe(ORB_ID(vehicle_control_flags));
	if (control_flags_sub < 0)
	{
		fprintf (stderr, "Waypoint manager thread failed to subscribe to vehicle_control_flags topic\n");
		exit(-1);
	}

	struct navigation_capabilities_s nav_cap;
	orb_subscr_t nav_cap_sub = orb_subscribe (ORB_ID(navigation_capabilities));
	if (nav_cap_sub == -1)
	{
		fprintf (stderr, "Waypoint manager thread failed to subscribe the navigation_capabilities topic\n");
		exit (-1);
	}

	struct parameter_update_s params;
	orb_subscr_t param_sub = orb_subscribe (ORB_ID(parameter_update));
	if (param_sub == -1)
	{
		fprintf (stderr, "Waypoint manager thread failed to subscribe the parameter_update topic\n");
		exit (-1);
	}

	orb_subscr_t mission_sub = orb_subscribe (ORB_ID(mission));
	if (mission_sub == -1)
	{
		fprintf (stderr, "Waypoint manager thread failed to subscribe the mission topic\n");
		exit (-1);
	}

	orb_subscr_t home_sub = orb_subscribe (ORB_ID(home_position));
	if (home_sub == -1)
	{
		fprintf (stderr, "Waypoint manager thread failed to subscribe the home_position topic\n");
		exit (-1);
	}


	/* abort on a nonzero return value from the parameter init */
	if (mission_waypoint_manager_params_init() != 0 || wp_manager_init (mission_sub, home_sub) != 0)
	{
		fprintf (stderr, "Waypoint manager exiting on startup due to an error\n");
		exit (-1);
	}

	turn_distance = (is_rotary_wing)? mission_waypoint_manager_parameters.mr_turn_distance :
									  mission_waypoint_manager_parameters.fw_turn_distance;

	/* welcome user */
	fprintf (stdout, "Waypoint manager started\n");
	fflush(stdout);

	/* waypoint main eventloop */
	while (!_shutdown_all_systems) {
		// wait for the position estimation for a max of 500ms
		updated = orb_poll (ORB_ID(vehicle_global_position), global_pos_sub, 500000);
		if (updated < 0)
		{
			// that's undesiderable but there is not much we can do
			fprintf (stderr, "Waypoint manager thread experienced an error waiting for actuator_controls topic\n");
			continue;
		}
		else if (updated == 0)
		{
			continue;
		}
		else
			orb_copy (ORB_ID(vehicle_global_position), global_pos_sub, (void *) &global_pos);

		updated = orb_check (ORB_ID(vehicle_control_flags), control_flags_sub);
		if (updated) {
			orb_copy(ORB_ID(vehicle_control_flags), control_flags_sub, &control_flags);
		}

		updated = orb_check (ORB_ID(navigation_capabilities), nav_cap_sub);
		if (updated)
		{
			orb_copy (ORB_ID(navigation_capabilities), nav_cap_sub, (void *) &nav_cap);
			turn_distance = (nav_cap.turn_distance > 0)? nav_cap.turn_distance : turn_distance;
		}

		updated = orb_check (ORB_ID(parameter_update), param_sub);
		if (updated) {
			/* clear updated flag */
			orb_copy(ORB_ID(parameter_update), param_sub, &params);

			/* update multirotor_position_control_parameters */
			mission_waypoint_manager_params_update();
		}

		if (!control_flags.flag_control_manual_enabled && control_flags.flag_control_auto_enabled)
		{
			orb_publish(ORB_ID(vehicle_global_position_setpoint), global_position_setpoint_pub, &g_sp);
			orb_publish(ORB_ID(vehicle_global_position_set_triplet), global_position_set_triplet_pub, &triplet);
		}

		now = get_absolute_time();

		/* check if waypoint has been reached against the last positions */
		check_waypoints_reached(now, &global_pos, turn_distance);

		/* sleep 25 ms */
		usleep(25000);
	}


	/*
	 * do unsubscriptions
	 */
	if (orb_unsubscribe (ORB_ID(vehicle_global_position), global_pos_sub, pthread_self()) < 0)
		fprintf (stderr, "Waypoint manager thread failed to unsubscribe to vehicle_global_position topic\n");

	if (orb_unsubscribe(ORB_ID(vehicle_control_flags), control_flags_sub, pthread_self()) < 0)
		fprintf (stderr, "Waypoint manager thread failed to unsubscribe to vehicle_control_flags topic\n");

	if (orb_unsubscribe (ORB_ID(navigation_capabilities), nav_cap_sub, pthread_self()) < 0)
		fprintf (stderr, "Waypoint manager thread failed to unsubscribe to navigation_capabilities topic\n");

	if (orb_unsubscribe (ORB_ID(parameter_update), param_sub, pthread_self()) < 0)
		fprintf (stderr, "Waypoint manager thread failed to unsubscribe to parameter_update topic\n");

	if (orb_unsubscribe (ORB_ID(mission), mission_sub, pthread_self()) < 0)
		fprintf (stderr, "Waypoint manager thread failed to unsubscribe to mission topic\n");

	if (orb_unsubscribe (ORB_ID(home_position), home_sub, pthread_self()) < 0)
		fprintf (stderr, "Waypoint manager thread failed to unsubscribe to home_position topic\n");

	/*
	 * do unadvertises
	 */
	if (orb_unadvertise (ORB_ID(vehicle_global_position_setpoint), global_position_setpoint_pub, pthread_self()) < 0)
		fprintf (stderr, "Waypoint manager thread failed to unadvertise the global_position_setpoint_pub topic\n");

	if (orb_unadvertise (ORB_ID(vehicle_global_position_set_triplet), global_position_set_triplet_pub, pthread_self()) < 0)
		fprintf (stderr, "Waypoint manager thread failed to unadvertise the vehicle_global_position_set_triplet topic\n");


	return 0;
}


