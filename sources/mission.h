// mission.h

#ifndef	_MISSION_H_
#define	_MISSION_H_

	#include <libxml/parser.h>
	#include "common.h"
	#include "mission_parameters.h"
	#include "mission_file_parser.h"

	#define ALWAYS_HAVE_A_MISSION
	#define ABORT_ON_MISSION_FILE_ERROR
	#define DEFAULT_MISSION_FILE_NAME	"dafault_mission.xml"
	
	
	typedef enum accepted_tag_t
	{
		accepted_tag_mission,
		accepted_tag_command,
		accepted_tag_control,
		accepted_tag_property
	} accepted_tag_t;
	
	inline static const char* accepted_tag_to_string (accepted_tag_t index)
    {
        switch(index)
        {
			return_custom_enum_string (accepted_tag_mission, "mission");
			return_custom_enum_string (accepted_tag_command, "command");
			return_custom_enum_string (accepted_tag_control, "control");
			return_custom_enum_string (accepted_tag_property, "property");
			default: return NULL;
        }
    }
	
	typedef enum mission_mode_t
	{
		mission_mode_resume,
		mission_mode_restart,
		mission_mode_N_MODES
	} mission_mode_t;
	
	inline static const char* mission_mode_to_string (mission_mode_t index)
    {
        switch(index)
        {
			return_custom_enum_string (mission_mode_resume, "resume");
			return_custom_enum_string (mission_mode_restart, "restart");
			default: return NULL;
        }
    }
	
	typedef enum mission_lastly_cmd_t
	{
		mission_lastly_cmd_do_nothing,
		mission_lastly_cmd_loiter,
		mission_lastly_cmd_rtl_and_land,
		mission_lastly_cmd_rtl_and_loiter,
		mission_lastly_cmd_N_COMMANDS
	} mission_lastly_cmd_t;
	
	inline static const char* mission_lastly_cmd_to_string (mission_lastly_cmd_t index)
    {
        switch(index)
        {
			return_custom_enum_string (mission_lastly_cmd_do_nothing, "do_nothing");
			return_custom_enum_string (mission_lastly_cmd_loiter, "loiter");
			return_custom_enum_string (mission_lastly_cmd_rtl_and_land, "rtl_and_land");
			return_custom_enum_string (mission_lastly_cmd_rtl_and_loiter, "rtl_and_loiter");
			default: return NULL;
        }
    }
		
	typedef enum condition_sign_t
	{
		condition_sign_equal,
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
		set_mode_absolute,
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
		loiter_mode_clockwise,
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
		set_variable_altitude,
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
		
	// maybe add some variables from autopilot.h::flight_parameters_error_t
	// maybe add some variables from autopilot_parameters.h::FDM & CONTROLS
	typedef enum test_variable_t
	{
		test_variable_altitude,
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
	
	
	typedef enum accepted_command_t
	{
		// commands
		accepted_command_waypoint,
		accepted_command_loiter,
		accepted_command_rtl,
		accepted_command_rth,
		accepted_command_takeoff,
		accepted_command_land,
		// controls
		accepted_command_delay,
		accepted_command_jump,
		accepted_command_set,
		accepted_command_repeat,
		accepted_command_end_repeat,
		accepted_command_while,
		accepted_command_end_while,
		accepted_command_if,
		accepted_command_end_if,
		// utils
		accepted_command_N_COMMANDS
	} accepted_command_t;
	
	inline static const char* accepted_command_to_string (accepted_command_t index)
    {
        switch(index)
        {
			// commands
			return_custom_enum_string (accepted_command_waypoint, "waypoint");
			return_custom_enum_string (accepted_command_loiter, "loiter");
			return_custom_enum_string (accepted_command_rtl, "rtl");
			return_custom_enum_string (accepted_command_rth, "rth");
			return_custom_enum_string (accepted_command_takeoff, "takeoff");
			return_custom_enum_string (accepted_command_land, "land");
			// controls
			return_custom_enum_string (accepted_command_delay, "delay");
			return_custom_enum_string (accepted_command_jump, "jump");
			return_custom_enum_string (accepted_command_set, "set");
			return_custom_enum_string (accepted_command_repeat, "repeat");
			return_custom_enum_string (accepted_command_end_repeat, "end_repeat");
			return_custom_enum_string (accepted_command_while, "while");
			return_custom_enum_string (accepted_command_end_while, "end_while");
			return_custom_enum_string (accepted_command_if, "if");
			return_custom_enum_string (accepted_command_end_if, "end_if");
			default: return NULL;
        }
    }
	
	struct mission_command_t;

	typedef union mission_option_t {
		double dbl;
		struct mission_command_t *cmd_ptr;
	} mission_option_t;
	
	typedef struct mission_command_t
	{
		accepted_command_t name;
		uint8_t id;
		uint8_t depth;
		float option1;
		double option2;
		mission_option_t option3;
		struct mission_command_t *next;
	} mission_command_t;
	
	typedef struct mission_t
	{
		mission_mode_t mode;				// What to do if the mission is interrupted and then resumed 
		mission_lastly_cmd_t lastly;		// What to do after the last command of the mission was completed
		
		mission_command_t *command_list;	// Mission commands
		mission_command_t *to_execute;		// Current command to execute
	} mission_t;
	
	
	/* function prototypes */
	int command_name_decode (const xmlChar *name, accepted_command_t *cmd_name);
	int test_variable_decode (const xmlChar *name, test_variable_t *variable_name);
	int set_variable_decode (const xmlChar *name, set_variable_t *variable_name);
	int set_mode_decode (const xmlChar *name, set_mode_t *set_mode);
	int mission_mode_decode (const xmlChar *name, mission_mode_t *mission_mode);
	int mission_lastly_cmd_decode (const xmlChar *name, mission_lastly_cmd_t *lastly_command);
	int loiter_mode_decode (const xmlChar *name, loiter_mode_t *loiter_mode);
	int condition_sign_decode (const xmlChar *name, condition_sign_t *cond);
	int check_coordinates (double latitude, double longitude);
	int check_command_altitude (accepted_command_t command_name, float altitude);
	int check_set_value (set_variable_t set_variable, double value);
	int buffer_print_mission_command (mission_command_t *command, char *buffer_ptr, int text_length);
	int mission_restart ();
	int mission_init (char *mission_file_name);
	void mission_destroy ();
	
	/* global variables */
	extern mission_t *mission;

#endif
