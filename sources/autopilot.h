// autopilot.h

#ifndef	_AUTOPILOT_H_
#define	_AUTOPILOT_H_

	#include "common.h"
	#include "autopilot_parameters.h"
	#include "mission_parameters.h"
	#include "mission.h"
	#include "aircraft_parameters.h"
	#include "aircraft.h"
	
	typedef enum flight_mode_t
	{
		flight_mode_manual,
		flight_mode_assist,
		flight_mode_stabilize,
		flight_mode_fly_by_wire,
		flight_mode_loiter,
		flight_mode_fly_to,
		flight_mode_rtl,
		flight_mode_rth,
		flight_mode_mission
	} flight_mode_t;
	
	static const char* flight_mode_to_string (flight_mode_t index)
    {
        switch(index)
        {
			return_custom_enum_string (flight_mode_manual, "manual");
			return_custom_enum_string (flight_mode_assist, "assist");
			return_custom_enum_string (flight_mode_stabilize, "stabilize");
			return_custom_enum_string (flight_mode_fly_by_wire, "fly_by_wire");
			return_custom_enum_string (flight_mode_loiter, "loiter");
			return_custom_enum_string (flight_mode_fly_to, "fly_to");
			return_custom_enum_string (flight_mode_rtl, "rtl");
			return_custom_enum_string (flight_mode_rth, "rth");
			return_custom_enum_string (flight_mode_mission, "mission");
			default: return NULL;
        }
    }
	
	typedef enum navigation_state_t
	{
		navigation_state_preflight,
		navigation_state_standby,
		navigation_state_armed,
		navigation_state_in_flight,
		navigation_state_warning_recover,
		navigation_state_mission_abort,
		navigation_state_emergency_landing,
		navigation_state_emergency_cutoff,
		navigation_state_ground_error,
		navigation_state_in_air_restore,
		navigation_state_reboot
	} navigation_state_t;

	static const char* navigation_state_to_string (navigation_state_t index)
    {
        switch(index)
        {
			return_custom_enum_string (navigation_state_preflight, "preflight");
			return_custom_enum_string (navigation_state_standby, "standby");
			return_custom_enum_string (navigation_state_armed, "armed");
			return_custom_enum_string (navigation_state_in_flight, "in_flight");
			return_custom_enum_string (navigation_state_warning_recover, "warning_recover");
			return_custom_enum_string (navigation_state_mission_abort, "mission_abort");
			return_custom_enum_string (navigation_state_emergency_landing, "emergency_landing");
			return_custom_enum_string (navigation_state_emergency_cutoff, "emergency_cutoff");
			return_custom_enum_string (navigation_state_ground_error, "ground_error");
			return_custom_enum_string (navigation_state_in_air_restore, "in_air_restore");
			return_custom_enum_string (navigation_state_reboot, "reboot");
			default: return NULL;
        }
    }
	
	typedef struct flight_parameters_error_t
	{
		float distance;
		float altitude_error;
		float bearing_error;
		float heading_error;
		float yaw_error, roll_error, pitch_error;
		float yaw_rate_error, roll_rate_error, pitch_rate_error;
		float north_velocity_error, east_velocity_error, down_velocity_error;
		float north_acceleration_error, east_acceleration_error, down_acceleration_error;
	} flight_parameters_error_t;

	typedef struct autopilot_t
	{
		navigation_state_t state;			// Aircraft navigation state
		flight_mode_t flight_mode;			// Aircraft flight mode
		flight_parameters_error_t* flight_parameters_error;	// Error between optimal flight parameters and current flight parameters
		
		map_point_t *home_WP;				// Coordinates of home point on the map
		map_point_t *takeoff_WP;			// Coordinates of map point where the aircraft has exceeded the takeoff height
		map_point_t *WP_to_reach;				// Coordinates of next point to reach on the map
	} autopilot_t;	
	
	
	/* function prototypes */
	int autopilot_init (char *mission_file_name);
	void autopilot_destroy ();
	int compute_flight_controls (double* flight_controls, double* flight_data);
	
	/* global variables */
	extern autopilot_t *autopilot;
	
#endif
