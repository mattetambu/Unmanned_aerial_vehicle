/**
 * @file mission_waypoint_manager_params.h
 *
 * Parameters defined for the mission waypoint manager
 *
 */


#ifndef __MISSION_WAYPOINT_MANAGER_PARAMS_H
#define __MISSION_WAYPOINT_MANAGER_PARAMS_H

	#include <stdint.h>
	#include "../uav_library/param/param.h"

	struct mission_waypoint_manager_parameters {
		float safe_loiter_altitude;
		float loiter_radius;
		float orbit_def_radius;
		float orbit_min_radius;
		float orbit_hold_time;
		float mr_turn_distance;
		float fw_turn_distance;
	} mission_waypoint_manager_parameters;			/**< local copies of interesting parameters */

	struct mission_waypoint_manager_parameter_handles {
		param_t safe_loiter_altitude;
		param_t loiter_radius;
		param_t orbit_def_radius;
		param_t orbit_min_radius;
		param_t orbit_hold_time;
		param_t fw_turn_distance;
		param_t mr_turn_distance;
	} mission_waypoint_manager_parameter_handles;		/**< handles for interesting parameters */


	/* function prototypes */
	int mission_waypoint_manager_params_define ();
	int mission_waypoint_manager_params_update ();
	int mission_waypoint_manager_params_init ();

	/* global variables */
	// empty

#endif
