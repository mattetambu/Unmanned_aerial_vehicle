/**
 * @file mission.h
 * Definition of the complete mission topic
 */

#ifndef TOPIC_MISSION_H_
#define TOPIC_MISSION_H_

	#include <stdint.h>
	#include "../ORB.h"
	
	#define ACCEPT_CONTROL_TAGS
	#define ACCEPT_ONLY_REPEAT_CONTROL_TAG	// WARNING: in order to enable all the control tags you must updated the find_next_setpoint_index and maybe others in mission_waypoint_manager.c


	/**
	 * @addtogroup topics
	 * @{
	 */

	#define MAX_COMMAND_IN_MISSION_LIST		25

	typedef enum accepted_tag_t
	{
		accepted_tag_mission = 0,
		accepted_tag_command,
#ifdef ACCEPT_CONTROL_TAGS
		accepted_tag_control,
#endif
		accepted_tag_property
	} accepted_tag_t;
	
	inline static const char* accepted_tag_to_string (accepted_tag_t index)
    {
        switch(index)
        {
			return_custom_enum_string (accepted_tag_mission, "mission");
			return_custom_enum_string (accepted_tag_command, "command");
#ifdef ACCEPT_CONTROL_TAGS
			return_custom_enum_string (accepted_tag_control, "control");
#endif
			return_custom_enum_string (accepted_tag_property, "property");
			default: return NULL;
        }
    }
	
	typedef enum mission_mode_t
	{
		mission_mode_resume = 0,
		//mission_mode_restart,
		mission_mode_N_MODES
	} mission_mode_t;
	
	inline static const char* mission_mode_to_string (mission_mode_t index)
    {
        switch(index)
        {
			return_custom_enum_string (mission_mode_resume, "resume");
			//return_custom_enum_string (mission_mode_restart, "restart");
			default: return NULL;
        }
    }
	
	typedef enum mission_lastly_cmd_t
	{
		mission_lastly_cmd_do_nothing = 0,
		mission_lastly_cmd_loiter,
		mission_lastly_cmd_land,
		mission_lastly_cmd_rtl,
		mission_lastly_cmd_N_COMMANDS
	} mission_lastly_cmd_t;
	
	inline static const char* mission_lastly_cmd_to_string (mission_lastly_cmd_t index)
    {
        switch(index)
        {
			return_custom_enum_string (mission_lastly_cmd_do_nothing, "do_nothing");
			return_custom_enum_string (mission_lastly_cmd_loiter, "loiter");
			return_custom_enum_string (mission_lastly_cmd_land, "land");
			return_custom_enum_string (mission_lastly_cmd_rtl, "rtl");
			default: return NULL;
        }
    }
		
	typedef enum condition_sign_t
	{
		condition_sign_equal = 0,
		condition_sign_not_equal,
		condition_sign_grater,
		condition_sign_grater_equal,
		condition_sign_less,
		condition_sign_less_equal,
		condition_sign_N_SIGNS
	} condition_sign_t;
	
	inline static const char* condition_sign_to_string (condition_sign_t index)
    {
        switch(index)
        {
			return_custom_enum_string (condition_sign_equal, "equal");
			return_custom_enum_string (condition_sign_not_equal, "not_equal");
			return_custom_enum_string (condition_sign_grater, "grater");
			return_custom_enum_string (condition_sign_grater_equal, "grater_equal");
			return_custom_enum_string (condition_sign_less, "less");
			return_custom_enum_string (condition_sign_less_equal, "less_equal");
			default: return NULL;
        }
    }
	
	inline static const char* condition_sign_to_simbol (condition_sign_t index)
    {
        switch(index)
        {
			return_custom_enum_string (condition_sign_equal, "=");
			return_custom_enum_string (condition_sign_not_equal, "!=");
			return_custom_enum_string (condition_sign_grater, ">");
			return_custom_enum_string (condition_sign_grater_equal, ">=");
			return_custom_enum_string (condition_sign_less, "<");
			return_custom_enum_string (condition_sign_less_equal, "<=");
			default: return NULL;
        }
    }
	
	typedef enum set_mode_t
	{
		set_mode_absolute = 0,
		set_mode_relative,
		set_mode_N_MODES
	} set_mode_t;
	
	inline static const char* set_mode_to_string (set_mode_t index)
    {
        switch(index)
        {
			return_custom_enum_string (set_mode_absolute, "absolute");
			return_custom_enum_string (set_mode_relative, "relative");
			default: return NULL;
        }
    }
	
	typedef enum loiter_mode_t
	{
		loiter_mode_clockwise = 0,
		loiter_mode_anticlockwise,
		loiter_mode_N_MODES
	} loiter_mode_t;
	
	inline static const char* loiter_mode_to_string (loiter_mode_t index)
    {
        switch(index)
        {
			return_custom_enum_string (loiter_mode_clockwise, "clockwise");
			return_custom_enum_string (loiter_mode_anticlockwise, "anticlockwise");
			default: return NULL;
        }
    }
		
	// maybe add some variables from autopilot_parameters.h::FDM & CONTROLS
	typedef enum set_variable_t
	{
		set_variable_altitude = 0,
		set_variable_speed,
		set_variable_N_VARIABLES
	} set_variable_t;
	
	inline static const char* set_variable_to_string (set_variable_t index)
    {
        switch(index)
        {
			return_custom_enum_string (set_variable_altitude, "altitude");
			return_custom_enum_string (set_variable_speed, "speed");
			default: return NULL;
        }
    }
		
	// maybe add some variables from flight_parameters_error_t
	// maybe add some variables from FlightGear_exchanged_data.h
	typedef enum test_variable_t
	{
		test_variable_altitude = 0,
		test_variable_speed,
		test_variable_heading_error,
		test_variable_wp_distance,
		test_variable_wp_eta,
		test_variable_N_VARIABLES
	} test_variable_t;
	
	inline static const char* test_variable_to_string (test_variable_t index)
    {
        switch(index)
        {
			return_custom_enum_string (test_variable_altitude, "altitude");
			return_custom_enum_string (test_variable_speed, "speed");
			return_custom_enum_string (test_variable_heading_error, "heading_error");
			return_custom_enum_string (test_variable_wp_distance, "wp_distance");
			return_custom_enum_string (test_variable_wp_eta, "wp_eta");
			default: return NULL;
        }
    }
	
	// WARNING - do not change order
	typedef enum accepted_command_t
	{
		// commands
		accepted_command_waypoint = 0,
		accepted_command_loiter_time,
		accepted_command_loiter_circle,
		accepted_command_loiter_unlim,
		accepted_command_rtl,
		accepted_command_takeoff,
		accepted_command_land,
#ifdef ACCEPT_CONTROL_TAGS
		// controls
#ifndef ACCEPT_ONLY_REPEAT_CONTROL_TAG
		accepted_command_delay,
		accepted_command_jump,
		accepted_command_set,
#endif
		accepted_command_repeat,
		accepted_command_end_repeat,
#ifndef ACCEPT_ONLY_REPEAT_CONTROL_TAG
		accepted_command_while,
		accepted_command_end_while,
		accepted_command_if,
		accepted_command_end_if,
#endif
#endif
		// utils
		accepted_command_N_COMMANDS
	} accepted_command_t;
	
	inline static const char* accepted_command_to_string (accepted_command_t index)
    {
        switch(index)
        {
			// commands
			return_custom_enum_string (accepted_command_waypoint, "waypoint");
			return_custom_enum_string (accepted_command_loiter_time, "loiter_time");
			return_custom_enum_string (accepted_command_loiter_circle, "loiter_circle");
			return_custom_enum_string (accepted_command_loiter_unlim, "loiter_unlim");
			return_custom_enum_string (accepted_command_rtl, "rtl");
			return_custom_enum_string (accepted_command_takeoff, "takeoff");
			return_custom_enum_string (accepted_command_land, "land");
#ifdef ACCEPT_CONTROL_TAGS
			// controls
#ifndef ACCEPT_ONLY_REPEAT_CONTROL_TAG
			return_custom_enum_string (accepted_command_delay, "delay");
			return_custom_enum_string (accepted_command_jump, "jump");
			return_custom_enum_string (accepted_command_set, "set");
#endif
			return_custom_enum_string (accepted_command_repeat, "repeat");
			return_custom_enum_string (accepted_command_end_repeat, "end_repeat");
#ifndef ACCEPT_ONLY_REPEAT_CONTROL_TAG
			return_custom_enum_string (accepted_command_while, "while");
			return_custom_enum_string (accepted_command_end_while, "end_while");
			return_custom_enum_string (accepted_command_if, "if");
			return_custom_enum_string (accepted_command_end_if, "end_if");
#endif
#endif
			default: return NULL;
        }
    }
	
	struct mission_command_t;

	
	/**
	 * Global position setpoint in WGS84 coordinates.
	 *
	 * This is the position the MAV is heading towards. If it of type loiter,
	 * the MAV is circling around it with the given loiter radius in meters.
	 */

	typedef uint32_t mission_command_index_t;
	
	typedef struct mission_command_t
	{
		accepted_command_t name;
		uint8_t id;
		uint8_t depth;
		uint8_t current; // 0, 1

		float option1;
		double option2;
		double option3;
		double option4;
		double option5;
	} mission_command_t;
	
	struct mission_s
	{
		mission_mode_t mode;				// What to do if the mission is interrupted and then resumed 
		mission_lastly_cmd_t lastly;		// What to do after the last command of the mission was completed

		mission_command_t command_list[MAX_COMMAND_IN_MISSION_LIST+1];	// Mission commands
		mission_command_index_t to_execute;	// Current command to execute
		mission_command_index_t n_commands;	// Number of commands
	};
	
	struct mission_small_s
	{
		//mission_mode_t mode;				// What to do if the mission is interrupted and then resumed
		//mission_lastly_cmd_t lastly;		// What to do after the last command of the mission was completed

		mission_command_t current_command;		// Mission active command
		mission_command_index_t command_index;	// Mission active command index
	};


	typedef struct mission_s mission_t;
	typedef struct mission_small_s mission_small_t;
	
	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(mission);
	ORB_DECLARE(mission_small);

#endif
