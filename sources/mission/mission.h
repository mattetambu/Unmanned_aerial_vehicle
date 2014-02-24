// mission.h

#ifndef	_MISSION_H_
#define	_MISSION_H_

	#include <libxml/parser.h>
	#include "../uav_library/common.h"
	#include "../ORB/topics/mission.h"


	#define ALWAYS_HAVE_A_MISSION
	#define ABORT_ON_MISSION_FILE_ERROR
	#define DEFAULT_MISSION_FILE_NAME	"dafault_mission.xml"
	
	// mission parameters
	// those parameters for now are only indicative
	#define DEFAULT_LOITER_RADIUS	15		//meters
	
	#define MIN_WAYPOINT_ALTITUDE	10		//meters
	#define MIN_LOITER_ALTITUDE		10		//meters
	#define MIN_LOITER_RADIUS		10		//meters
	#define MIN_RTL_ALTITUDE		10		//meters
	#define MIN_TAKEOFF_ALTITUDE	10		//meters
	#define MIN_LAND_ALTITUDE		10		//meters
	
	#define MAX_WAYPOINT_ALTITUDE	100		//meters
	#define MAX_LOITER_ALTITUDE		100		//meters
	#define MAX_RTL_ALTITUDE		100		//meters
	#define MAX_TAKEOFF_ALTITUDE	100		//meters
	#define MAX_LAND_ALTITUDE		100		//meters
	
	#define MIN_SET_ALTITUDE		10		//meters
	#define MAX_SET_ALTITUDE		100		//meters
	#define MIN_SET_SPEED			10		//meters/seconds
	#define MAX_SET_SPEED			100		//meters/seconds
	
	#define MIN_LOITER_TIME			10		//seconds
	#define MIN_LOITER_ROUNDS		1		//complete rounds
	
		
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
	int mission_restart ();
	int mission_init (char *mission_file_name);
	void mission_destroy ();
	
	/* global variables */
	// empty

#endif
