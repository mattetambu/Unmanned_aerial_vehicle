/**
 * @file mission_waypoint_manager_params.c
 *
 * Parameters defined for the mission waypoint manager
 *
 */


#include "../uav_library/param/param.h"
#include "mission_waypoint_manager_params.h"


/*
 * Controller parameters
 *
 */
int mission_waypoint_manager_params_define ()
{
	PARAM_DEFINE_FLOAT(SAFE_LOITER_ALT, 15.0f);
	PARAM_DEFINE_FLOAT(LOITER_RADIUS, 10.0f);
	PARAM_DEFINE_FLOAT(ORBIT_DEF_RADIUS, 10.0f);
	PARAM_DEFINE_FLOAT(ORBIT_MIN_RADIUS, 3.5f);
	PARAM_DEFINE_FLOAT(ORBIT_HOLD_TIME, 1.0f);
	PARAM_DEFINE_FLOAT(FW_TURN_DIST, 7.5f);
	PARAM_DEFINE_FLOAT(MR_TURN_DIST, 2.0f);

	return 0;
}

int mission_waypoint_manager_params_update()
{
	PARAM_GET (mission_waypoint_manager_parameter_handles.safe_loiter_altitude, &(mission_waypoint_manager_parameters.safe_loiter_altitude));
	PARAM_GET (mission_waypoint_manager_parameter_handles.loiter_radius, &(mission_waypoint_manager_parameters.loiter_radius));
	PARAM_GET (mission_waypoint_manager_parameter_handles.orbit_def_radius, &(mission_waypoint_manager_parameters.orbit_def_radius));
	PARAM_GET (mission_waypoint_manager_parameter_handles.orbit_min_radius, &(mission_waypoint_manager_parameters.orbit_min_radius));
	PARAM_GET (mission_waypoint_manager_parameter_handles.orbit_hold_time, &(mission_waypoint_manager_parameters.orbit_hold_time));
	PARAM_GET (mission_waypoint_manager_parameter_handles.fw_turn_distance, &(mission_waypoint_manager_parameters.fw_turn_distance));
	PARAM_GET (mission_waypoint_manager_parameter_handles.mr_turn_distance, &(mission_waypoint_manager_parameters.mr_turn_distance));

	return 0;
}


int mission_waypoint_manager_params_init ()
{
	/* wp manager parameters */
	mission_waypoint_manager_parameter_handles.safe_loiter_altitude =	param_find("SAFE_LOITER_ALT");
	mission_waypoint_manager_parameter_handles.loiter_radius		=	param_find("LOITER_RADIUS");
	mission_waypoint_manager_parameter_handles.orbit_def_radius		=	param_find("ORBIT_DEF_RADIUS");
	mission_waypoint_manager_parameter_handles.orbit_min_radius		=	param_find("ORBIT_MIN_RADIUS");
	mission_waypoint_manager_parameter_handles.orbit_hold_time		= 	param_find("ORBIT_HOLD_TIME");
	mission_waypoint_manager_parameter_handles.fw_turn_distance		= 	param_find("FW_TURN_DIST");
	mission_waypoint_manager_parameter_handles.mr_turn_distance 	= 	param_find("MR_TURN_DIST");


	return mission_waypoint_manager_params_update();
}
