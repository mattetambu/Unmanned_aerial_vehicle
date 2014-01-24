// mission.c

#include "mission.h"


mission_t *mission = NULL;

int command_name_decode (const xmlChar *name, accepted_command_t *cmd_name)
{
	accepted_command_t command_name;
	if (name == NULL)
		return -1;
	
	for (command_name = 0; command_name < accepted_command_N_COMMANDS; command_name++)
		if (!strcmp((const char *) name, accepted_command_to_string(command_name)))
		{
			*cmd_name = command_name;
			return 0;
		}
		
	return -1;
}

int test_variable_decode (const xmlChar *name, test_variable_t *variable_name)
{
	test_variable_t test_variable;
	if (name == NULL)
		return -1;
	
	for (test_variable = 0; test_variable < test_variable_N_VARIABLES; test_variable++)
		if (!strcmp((const char *) name, test_variable_to_string(test_variable)))
		{
			*variable_name = test_variable;
			return 0;
		}
		
	return -1;
}

int set_variable_decode (const xmlChar *name, set_variable_t *variable_name)
{
	set_variable_t set_variable;
	if (name == NULL)
		return -1;
	
	for (set_variable = 0; set_variable < set_variable_N_VARIABLES; set_variable++)
		if (!strcmp((const char *) name, set_variable_to_string(set_variable)))
		{
			*variable_name = set_variable;
			return 0;
		}
		
	return -1;
}

int set_mode_decode (const xmlChar *name, set_mode_t *set_mode)
{
	set_mode_t mode;
	if (name == NULL)
		return -1;
	
	for (mode = 0; mode < set_mode_N_MODES; mode++)
		if (!strcmp((const char *) name, set_mode_to_string(mode)))
		{
			*set_mode = mode;
			return 0;
		}
		
	return -1;
}

int mission_mode_decode (const xmlChar *name, mission_mode_t *mission_mode)
{
	mission_mode_t mode;
	if (name == NULL)
		return -1;
	
	for (mode = 0; mode < mission_mode_N_MODES; mode++)
		if (!strcmp((const char *) name, mission_mode_to_string(mode)))
		{
			*mission_mode = mode;
			return 0;
		}
		
	return -1;
}

int mission_lastly_cmd_decode (const xmlChar *name, mission_lastly_cmd_t *lastly_command)
{
	mission_lastly_cmd_t command;
	if (name == NULL)
		return -1;
	
	for (command = 0; command < mission_lastly_cmd_N_COMMANDS; command++)
		if (!strcmp((const char *) name, mission_lastly_cmd_to_string(command)))
		{
			*lastly_command = command;
			return 0;
		}
		
	return -1;
}

int loiter_mode_decode (const xmlChar *name, loiter_mode_t *loiter_mode)
{
	loiter_mode_t mode;
	if (name == NULL)
		return -1;
	
	for (mode = 0; mode < loiter_mode_N_MODES; mode++)
		if (!strcmp((const char *) name, loiter_mode_to_string(mode)))
		{
			*loiter_mode = mode;
			return 0;
		}
		
	return -1;
}

int condition_sign_decode (const xmlChar *name, condition_sign_t *cond)
{
	condition_sign_t condition;
	if (name == NULL)
		return -1;
	
	for (condition = 0; condition < condition_sign_N_SIGNS; condition++)
		if (!strcmp((const char *) name, condition_sign_to_string(condition)))
		{
			*cond = condition;
			return 0;
		}
		
	return -1;
}

int check_coordinates (double latitude, double longitude)
{
	if (latitude < -90 || latitude > 90)
	{
		fprintf (stderr, "Invalid property \'latitude\' in command tag of type \'waypoint\'\n");
		fprintf (stderr, "It must be a real number between -90 and 90 degrees\n");
		return -1;
	}
	if (longitude < -180 || longitude > 180)
	{
		fprintf (stderr, "Invalid property \'latitude\' in command tag of type \'waypoint\'\n");
		fprintf (stderr, "It must be a real number between -180 and 180 degrees\n");
		return -1;
	}
	
	return 0;
}

int check_command_altitude (accepted_command_t command_name, float altitude)
{
	switch (command_name)
	{
		case accepted_command_rtl:
#ifdef MIN_RTL_ALTITUDE
			if (altitude < MIN_RTL_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number grater than %d meters\n", MIN_RTL_ALTITUDE);
				return -1;
			}
#endif
#ifdef MAX_RTL_ALTITUDE
			if (altitude > MAX_RTL_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number less than %d meters\n", MAX_RTL_ALTITUDE);
				return -1;
			}
#endif
			break;
		case accepted_command_rth:
#ifdef MIN_RTH_ALTITUDE
			if (altitude < MIN_RTH_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number grater than %d meters\n", MIN_RTH_ALTITUDE);
				return -1;
			}
#endif
#ifdef MAX_RTH_ALTITUDE
			if (altitude > MAX_RTH_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number less than %d meters\n", MAX_RTH_ALTITUDE);
				return -1;
			}
#endif
			break;

		case accepted_command_takeoff:
#ifdef MIN_TAKEOFF_ALTITUDE
			if (altitude < MIN_TAKEOFF_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number grater than %d meters\n", MIN_TAKEOFF_ALTITUDE);
				return -1;
			}
#endif
#ifdef MAX_TAKEOFF_ALTITUDE
			if (altitude > MAX_TAKEOFF_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number less than %d meters\n", MAX_TAKEOFF_ALTITUDE);
				return -1;
			}
#endif
			break;

		case accepted_command_land:
#ifdef MIN_LAND_ALTITUDE
			if (altitude < MIN_LAND_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number grater than %d meters\n", MIN_LAND_ALTITUDE);
				return -1;
			}
#endif
#ifdef MAX_LAND_ALTITUDE
			if (altitude > MAX_LAND_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number less than %d meters\n", MAX_LAND_ALTITUDE);
				return -1;
			}
#endif
			break;

		case accepted_command_waypoint:
#ifdef MIN_WAYPOINT_ALTITUDE
			if (altitude < MIN_WAYPOINT_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number grater than %d meters\n", MIN_WAYPOINT_ALTITUDE);
				return -1;
			}
#endif
#ifdef MAX_WAYPOINT_ALTITUDE
			if (altitude > MAX_WAYPOINT_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number less than %d meters\n", MAX_WAYPOINT_ALTITUDE);
				return -1;
			}
#endif
			break;

		case accepted_command_loiter:
#ifdef MIN_LOITER_ALTITUDE
			if (altitude < MIN_LOITER_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number grater than %d meters\n", MIN_LOITER_ALTITUDE);
				return -1;
			}
#endif
#ifdef MAX_LOITER_ALTITUDE
			if (altitude > MAX_LOITER_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'altitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command_name));
				fprintf (stderr, "It must be a positive number less than %d meters\n", MAX_LOITER_ALTITUDE);
				return -1;
			}
#endif
			break;

		default:
			// Command name not accepted in command tag
			fprintf (stderr, "Invalid property \'name\' for command tag\n");
			fprintf (stderr, "Accepted values:\t[waypoint,loiter,rtl,rth,takeoff,land]\n");
			return -1;
	}
	
	return 0;
}

int check_set_value (set_variable_t set_variable, double value)
{
	switch (set_variable)
	{
		case set_variable_altitude:
#ifdef MIN_SET_ALTITUDE
			if (value < MIN_SET_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'value\' in command tag of type \'set\'\n");
				fprintf (stderr, "Altitude value must be a positive number grater than %d meters\n", MIN_SET_ALTITUDE);
				return -1;
			}
#endif
#ifdef MAX_SET_ALTITUDE
			if (value > MAX_SET_ALTITUDE)
			{
				fprintf (stderr, "Invalid property \'value\' in command tag of type \'set\'\n");
				fprintf (stderr, "Altitude value must be a positive number less than %d meters\n", MAX_SET_ALTITUDE);
				return -1;
			}
#endif
			break;
		case set_variable_speed:
#ifdef MIN_SET_SPEED
			if (value < MIN_SET_SPEED)
			{
				fprintf (stderr, "Invalid property \'value\' in command tag of type \'set\'\n");
				fprintf (stderr, "Speed value must be a positive number grater than %d meters//seconds\n", MIN_SET_SPEED);
				return -1;
			}
#endif
#ifdef MAX_SET_SPEED
			if (value > MAX_SET_SPEED)
			{
				fprintf (stderr, "Invalid property \'value\' in command tag of type \'set\'\n");
				fprintf (stderr, "Speed value must be a positive number less than %d meters//seconds\n", MAX_SET_SPEED);
				return -1;
			}
#endif
			break;
		default:
			// should not happen
			return -1;
	}

	return 0;
}


int buffer_print_mission_command (mission_command_t *command, char *buffer_ptr, int text_length)
{
	int i, j, return_value;

	// should not happen
	if (!command)
		return -1;
	
	memset(buffer_ptr, '\0', text_length);

	for (i = 0; i < ((int) command->depth*N_SPACES_PER_TAB) && i < text_length;)
		for (j = 0; j < N_SPACES_PER_TAB; j++)
			buffer_ptr[i++] = ' ';
	
	
	if (command->name == accepted_command_end_repeat ||
		command->name == accepted_command_end_while ||
		command->name == accepted_command_end_if)
		{
			return_value = snprintf (buffer_ptr+i, text_length-i, "  }\n");
			return (return_value < 0)? -1 : i + return_value;
		}
	
	return_value = (command->name == accepted_command_repeat)? 
					snprintf (buffer_ptr+i, text_length-i, "  while  ") :
					snprintf (buffer_ptr+i, text_length-i, "  %s  ", accepted_command_to_string(command->name));
	i += return_value;
	if (return_value < 0)
		return -1;
	
	if (command->id > 0)
	{
		return_value = snprintf (buffer_ptr+i, text_length-i, "id=%d  ", command->id);
		i += return_value;
		if (return_value < 0)
			return -1;
	}
	
	switch (command->name)
	{
		case accepted_command_rtl:
		case accepted_command_rth:
		case accepted_command_takeoff:
		case accepted_command_land:
			if (command->option1 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.3f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			break;
		case accepted_command_waypoint:
			if (command->option1 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.3f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			
			return_value = snprintf (buffer_ptr+i, text_length-i, "latitude=%.8f  longitude=%.8f  ", command->option2, command->option3.dbl);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_loiter:
			if (command->option1 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.3f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			if (command->option2 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "seconds=%.1f  ", command->option2);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			else if (command->option2 < 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "rounds=%.0f  ", -command->option2);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			
			return_value = snprintf (buffer_ptr+i, text_length-i, "mode=%s  ", loiter_mode_to_string((loiter_mode_t) command->option3.dbl));
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_delay:
			return_value = snprintf (buffer_ptr+i, text_length-i, "seconds=%.1f  ", command->option2);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_jump:
			return_value = snprintf (buffer_ptr+i, text_length-i, "target_id=%.0f  ", command->option2);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_set:
			return_value = snprintf (buffer_ptr+i, text_length-i, "variable=%s  value=%.3f  mode=%s  ",
									set_variable_to_string((set_variable_t) command->option1),
									command->option2,
									set_mode_to_string((set_mode_t) command->option3.dbl));
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_repeat:
			return_value = snprintf (buffer_ptr+i, text_length-i, "(counter  <  %.0f)  {  ", command->option2);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
		
		case accepted_command_while:
		case accepted_command_if:
			return_value = snprintf (buffer_ptr+i, text_length-i, "(%s  %s  %.3f)  {  ",
									test_variable_to_string ((test_variable_t) command->option2),
									condition_sign_to_simbol ((condition_sign_t) command->option3.cmd_ptr->option1),
									command->option3.cmd_ptr->option2);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		default:
			// should not happen
			fprintf (stderr, "Invalid property \'name\' in a command in the mission command list\n");
			return -1;
		
	}
	return_value = snprintf (buffer_ptr+i, text_length-i, "\n");

	return (return_value < 0)? -1 : i + return_value;
}

int mission_restart ()
{
	// should not happen
	if (mission == NULL)
	{
		fprintf (stderr, "Mission list not initialized\n");
		return -1;
	}
	
	mission->to_execute = mission->command_list;
	
	fprintf (stdout, "Mission restarted\n");
	fflush (stdout);
	
	return 0;
}

int mission_init (char *mission_file_name)
{
	xmlDoc *mission_file = NULL;	// Mission file
	xmlNode *root_node;
	
	mission = malloc (sizeof(mission_t));
	if (mission == NULL)
	{
		fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
		return -1;
	}
	memset (mission, 0, sizeof (mission_t));
	
	if (getenv("MISSION_SET") && (mission_file = xmlReadFile (mission_file_name, NULL, 0)) == NULL)
	{
#ifdef ABORT_ON_MISSION_FILE_ERROR
		fprintf (stderr, "Error reading the mission file %s\n", mission_file_name);
		return -1;
#else
		fprintf (stderr, "Error reading the specified mission file %s\n", mission_file_name);
#endif
	}
	
	if (mission_file == NULL)
	{
#ifdef ALWAYS_HAVE_A_MISSION
		fprintf (stderr, "The default mission will be used\n");
		mission_file = xmlReadFile (DEFAULT_MISSION_FILE_NAME, NULL, 0);
		if (mission_file == NULL)
		{
			fprintf (stderr, "Default mission file corrupted or not found\n");
			return -1;
		}
#else
		fprintf (stdout, "No mission set for this execution\n")
#endif
	}
	
	root_node = xmlDocGetRootElement (mission_file);
	if (root_node == NULL || process_mission_node (root_node) < 0)
	{
		fprintf (stderr, "Can't process the mission node\n");
		return -1;
	}
		
	if (mission_file_parse (/*mission, */root_node, 0 /*node depth*/) < 0)
	{
		fprintf (stderr, "Failed to parse the mission file\n");
		return -1;
	}
	mission->to_execute = mission->command_list;
	
	if (check_mission_integrity () < 0)
	{
		// already printed an error message
		return -1;
	}
	
	/* free the document */
	xmlFreeDoc(mission_file);
	
	/*
	 * Cleanup function for the XML library.
	 */
	xmlCleanupParser();
	
	if (getenv("VERBOSE"))
	{
		fprintf (stdout, "Mission file successfully parsed\n");
		fflush (stdout);
	}
	
	return 0;
}

void mission_destroy ()
{
	mission_command_t *command, *to_destroy;
	
	if (!mission)
		return;
		
	command = mission->command_list;
	
	/*free the memory */
	while (command)
	{
		to_destroy = command;
		command = command->next;
		free (to_destroy);
	}
	free(mission);
}
