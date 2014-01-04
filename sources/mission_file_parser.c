// mission_file_parser.c

#include "common.h"
#include "mission_file_parser.h"
#include "mission_logic.h"
#include "mission_parameters.h"

typedef struct mission_initialize_support_t
{
	accepted_command_t name;
	int line_counter;		//for jump node means the command position in the mission_list
							//for if, while and repeat node means the total child count
	mission_command_t *command;
	struct mission_initialize_support_t *next;
} mission_initialize_support_t;

typedef struct command_property_t
{
	xmlChar *name;
	xmlChar *value;
} command_property_t;


mission_initialize_support_t *support_list = NULL;
int mission_n_children = 0;


int process_property_node (xmlNode *node, command_property_t *property)
{
	if (!xmlHasProp (node, (const xmlChar *) "name"))
	{
		fprintf (stderr, "Required property \'name\' for every \'property\' tag\n");
		return -1;
	}
	property->name = xmlGetProp (node, (const xmlChar *) "name");

	if (!xmlHasProp (node, (const xmlChar *) "value"))
	{
		fprintf (stderr, "Required property \'value\' for every \'property\' tag\n");
		return -1;
	}
	property->value = xmlGetProp (node, (const xmlChar *) "value");

	return 0;
}

int xml_GetProp (xmlNode *current_node, const char *name, xmlChar **prop)
{
	xmlNode *node = NULL;
	command_property_t property;
	//memset (&property, 0, sizeof (command_property_t));
	
	if (xmlHasProp (current_node, (const xmlChar *) name))
	{
		*prop = xmlGetProp (current_node, (const xmlChar *) name);
		return 0;
	}
	
	// look if there is a child of type property whit the desidered property
	for (node = current_node->children; node; node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE ||
			xmlNodeIsText(node) ||
			strcmp((const char *) node->name, accepted_tag_to_string(accepted_tag_property)) != 0)
			continue;
		
		if (process_property_node (node, &property) < 0)
		{
			fprintf (stderr, "Unable to parse a property node\n");
			break;
		}
		
		if (!strcmp(name, (const char *) property.name))
		{
			*prop = property.value;
			return 0;
		}
	}
	
	return -1;
}

int process_mission_node (xmlNode *node)
{
	xmlChar *property = NULL;
	
	if (xml_GetProp (node, "mode", &property) >= 0)
	{
		if (mission_mode_decode (property, &mission->mode) < 0)
		{
			fprintf (stderr, "Invalid property \'mode\' in mission tag\n");
			fprintf (stderr, "Accepted values:\t[restart,resume]\n");
			return -1;
		}
	}
	else mission->mode = mission_mode_resume;
		
	if (xml_GetProp (node, "lastly", &property) >= 0)
	{
		if (mission_lastly_cmd_decode (property, &mission->lastly) < 0)
		{
			fprintf (stderr, "Invalid property \'lastly\' in mission tag\n");
			fprintf (stderr, "Accepted values:\t[do_nothing,loiter,rtl_and_land,rtl_and_loiter]\n");
			return -1;
		}
	}
	else mission->lastly = mission_lastly_cmd_do_nothing;
	
	return 0;
}

int process_command_node (xmlNode *node, int node_depth, mission_command_t *command)
{
	xmlChar *property = NULL;
	loiter_mode_t loiter_mode;
	
	if (xml_GetProp (node, "id", &property) >= 0)
	{
		command->id = (uint8_t) atoi ((const char *) property);
		if (command->id <= 0)
		{
			fprintf (stderr, "Invalid property \'id\' in command tag\n");
			fprintf (stderr, "It must be a not-zero positive integer\n");
			return -1;
		}
	}
	
	command->depth = node_depth;
	
	if (xml_GetProp (node, "name", &property) < 0)
	{
		fprintf (stderr, "Required property \'name\' for every \'command\' tag\n");
		return -1;
	}
	if (command_name_decode (property, &command->name) < 0)
	{
		fprintf (stderr, "Invalid property \'name\' in command tag\n");
		fprintf (stderr, "Accepted values:\t[waypoint,loiter,rtl,rth,takeoff,land]\n");
		return -1;
	}
	
	if (xml_GetProp (node, "altitude", &property) >= 0)
	{
		command->option1 = (float) atof ((const char *) property);
		if (check_command_altitude (command->name, command->option1) < 0)
		{
			// already printed an error message
			return -1;
		}
	}
	
	switch (command->name)
	{
		case accepted_command_rtl:
		case accepted_command_rth:
		case accepted_command_takeoff:
		case accepted_command_land:
			break;
			
		case accepted_command_waypoint:
			if (xml_GetProp (node, "latitude", &property) < 0)
			{
				fprintf (stderr, "Required property \'latitude\' for command tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			command->option2 = (double) atof ((const char *) property);
			if (check_coordinates (command->option2) < 0)
			{
				fprintf (stderr, "Invalid property \'latitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command->name));
				fprintf (stderr, "It must be a positive number grater than 0\n"); // TO BE VERIFIED
				return -1;
			}
			
			if (xml_GetProp (node, "longitude", &property) < 0)
			{
				fprintf (stderr, "Required property \'longitude\' for command tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			command->option3.dbl = (double) atof ((const char *) property);
			if (check_coordinates (command->option3.dbl) < 0)
			{
				fprintf (stderr, "Invalid property \'longitude\' in command tag of type \'%s\'\n", accepted_command_to_string(command->name));
				fprintf (stderr, "It must be a positive number grater than 0\n"); // TO BE VERIFIED
				return -1;
			}
			break;
			
		case accepted_command_loiter:
			if (xml_GetProp (node, "mode", &property) >= 0)
			{
				if (loiter_mode_decode (property, &loiter_mode) < 0)
				{
					fprintf (stderr, "Invalid property \'mode\' in command tag of type \'%s\'\n", accepted_command_to_string(command->name));
					fprintf (stderr, "Accepted values:\t[clockwise,anticlockwise]\n");
					return -1;
				}
			}
			command->option3.dbl = (double) loiter_mode;
			
			if (xml_GetProp (node, "seconds", &property) >= 0)
			{
				command->option2 = (float) atof ((const char *) property);
				if (command->option2 <= MIN_LOITER_TIME)
				{
					fprintf (stderr, "Invalid property \'seconds\' in command tag of type \'%s\'\n", accepted_command_to_string(command->name));
					fprintf (stderr, "It must be a positive number grater than %d seconds\n", MIN_LOITER_TIME);
					return -1;
				}
			}
			else if (xml_GetProp (node, "rounds", &property) >= 0)
			{
				command->option2 = (float) atof ((const char *) property);
				if (command->option2 <= MIN_LOITER_ROUNDS)
				{
					fprintf (stderr, "Invalid property \'rounds\' in command tag of type \'%s\'\n", accepted_command_to_string(command->name));
					fprintf (stderr, "It must be a positive number grater than %d rounds\n", MIN_LOITER_ROUNDS);
					return -1;
				}
			}
			break;
			
		default:
			// Command name not accepted in command tag
			fprintf (stderr, "Invalid property \'name\' for command tag\n");
			fprintf (stderr, "Accepted values:\t[waypoint,loiter,rtl,rth,takeoff,land]\n");
			return -1;
	}

	return 0;
}

int process_control_node (xmlNode *node, int node_depth, mission_command_t *command)
{
	test_variable_t test_variable;
	set_variable_t set_variable;
	set_mode_t set_mode;
	condition_sign_t condition;
	xmlChar *property = NULL;
	
	if (xml_GetProp (node, "id", &property) >= 0)
	{
		command->id = (uint8_t) atoi ((const char *) property);
		if (command->id <= 0)
		{
			fprintf (stderr, "Invalid property \'id\' in command tag\n");
			fprintf (stderr, "It must be a not-zero positive integer\n");
			return -1;
		}
	}
	
	command->depth = node_depth;
	
	if (xml_GetProp (node, "name", &property) < 0)
	{
		fprintf (stderr, "Required property \'name\' for every \'control\' tag\n");
		return -1;
	}
	if (command_name_decode (property, &command->name) < 0)
	{
		fprintf (stderr, "Invalid property \'name\' in control tag\n");
		fprintf (stderr, "Accepted values:\t[delay,jump,set,repeat,while,if]\n");
		return -1;
	}
	
	switch (command->name)
	{
		case accepted_command_while:
		case accepted_command_if:
			if (xml_GetProp (node, "variable", &property) < 0)
			{
				fprintf (stderr, "Required property \'variable\' for control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			if (test_variable_decode (property, &test_variable) < 0)
			{
				fprintf (stderr, "Invalid property \'variable\' in control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				fprintf (stderr, "Accepted values:\t[altitude,speed,heading_error,wp_distance]\n");
				return -1;
			}
			command->option1 = (double) test_variable;
	
			if (xml_GetProp (node, "value", &property) < 0)
			{
				fprintf (stderr, "Required property \'value\' for control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			command->option2 = (double) atof ((const char *) property);
			
			if (xml_GetProp (node, "condition", &property) < 0)
			{
				fprintf (stderr, "Required property \'condition\' for control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			if (condition_sign_decode (property, &condition) < 0)
			{
				fprintf (stderr, "Invalid property \'condition\' in control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				fprintf (stderr, "Accepted values:\t[equal,not_equal,grater,grater_equal,less,less_equal]\n");
				return -1;
			}
			command->option3.dbl = (double) condition;
			break;

		case accepted_command_set:
			if (xml_GetProp (node, "variable", &property) < 0)
			{
				fprintf (stderr, "Required property \'variable\' for control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			if (set_variable_decode (property, &set_variable) < 0)
			{
				fprintf (stderr, "Invalid property \'variable\' in control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				fprintf (stderr, "Accepted values:\t[altitude,speed]\n");
				return -1;
			}
			command->option1 = (double) set_variable;
			
			if (xml_GetProp (node, "value", &property) < 0)
			{
				fprintf (stderr, "Required property \'value\' for control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			command->option2 = (double) atof ((const char *) property);
			
			if (xml_GetProp (node, "mode", &property) >= 0)
			{
				if (set_mode_decode (property, &set_mode) < 0)
				{
					fprintf (stderr, "Invalid property \'mode\' in control tag of type \'%s\'\n", accepted_command_to_string(command->name));
					fprintf (stderr, "Accepted values:\t[absolute,relative]\n");
					return -1;
				}
			}
			command->option3.dbl = (double) set_mode;
			break;		
	
		case accepted_command_delay:
			if (xml_GetProp (node, "seconds", &property) < 0)
			{
				fprintf (stderr, "Required property \'seconds\' for control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			command->option2 = (double) atof ((const char *) property);
			if (command->option2 <= 0)
			{
				fprintf (stderr, "Invalid property \'seconds\' in control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				fprintf (stderr, "It must be a not-zero positive integer\n");
				return -1;
			}
			break;
			
		case accepted_command_jump:
			if (xml_GetProp (node, "target", &property) < 0)
			{
				fprintf (stderr, "Required property \'target\' for control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			command->option2 = (double) atof ((const char *) property);
			if (command->option2 <= 0)
			{
				fprintf (stderr, "Invalid property \'target\' in control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				fprintf (stderr, "It must be a not-zero positive integer\n");
				return -1;
			}
			break;
			
		case accepted_command_repeat:
			if (xml_GetProp (node, "times", &property) < 0)
			{
				fprintf (stderr, "Required property \'times\' for control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				return -1;
			}
			command->option2 = (double) atof ((const char *) property);
			if (command->option2 <= 0)
			{
				fprintf (stderr, "Invalid property \'times\' in control tag of type \'%s\'\n", accepted_command_to_string(command->name));
				fprintf (stderr, "It must be a not-zero positive integer\n");
				return -1;
			}
			break;
			
		default:
			// Command name not accepted in control tag
			fprintf (stderr, "Invalid property \'name\' for control tag\n");
			fprintf (stderr, "Accepted values:\t[delay,jump,set,repeat,while,if]\n");
			return -1;
	}

	return 0;
}


int insert_command_in_mission_list (mission_command_t *cmd)
{
	mission_initialize_support_t *support_elem = NULL;

	// should not happen
	/*if (mission == NULL)
	{
		fprintf (stderr, "Mission list not initialized\n");
		return -1;
	}

	if (cmd == NULL)
	{
		fprintf (stderr, "Nothing to insert in the mission list - Empty command element\n");
		return -1;
	}*/

	if (mission->to_execute != NULL)
	{
		//add a command on the tail of the mission list
		mission->to_execute->next = malloc (sizeof(mission_command_t));
		mission->to_execute = mission->to_execute->next;
	}
	else
	{	//add a command on the head of the mission list
		mission->command_list = malloc (sizeof(mission_command_t));
		mission->to_execute = mission->command_list;
	}

	if (mission->to_execute == NULL)
	{
		fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
		return -1;
	}
	memset (mission->to_execute, 0, sizeof (mission_command_t));
	*(mission->to_execute) = *cmd;


	// update the state variables
	mission_n_children++;
	
	for (support_elem = support_list; support_elem != NULL && support_elem->name != accepted_command_jump; support_elem = support_elem->next)
			support_elem->line_counter++;

	return 0;
}

int insert_command_end_in_mission_list (uint8_t command_depth)
{
	mission_initialize_support_t *support_elem = NULL;
	mission_command_t command_end;
	memset (&command_end, 0, sizeof (mission_command_t));
	
	// no need to insert the end tag
	if (support_list == NULL ||
		support_list->command->name == accepted_command_jump ||
		command_depth > support_list->command->depth)
		return 0;

	/*// should not happen
	if (support_list->command == NULL)
	{
		fprintf (stderr, "Can't find the reference to the command opening tag - Empty command element\n");
		return -1;
	}*/
	
	switch (support_list->command->name)
	{
		case accepted_command_jump:
			return 0;
		case accepted_command_while:
		case accepted_command_if:
			command_end.option1 = (float) support_list->command->option3.dbl;
			support_list->command->option2 = (double) support_list->command->option1;
			support_list->command->option1 = (float) support_list->line_counter;
			command_end.option2 = (double) support_list->command->option2;
			command_end.option3.cmd_ptr = support_list->command;
			break;

		case accepted_command_repeat:
			support_list->command->option1 = (float) support_list->line_counter;
			command_end.option2 = (double) support_list->command->option2;
			command_end.option3.cmd_ptr = support_list->command;
			break;

		default:
			// Should not happen
			fprintf (stderr, "Error in a control tag of type \'%s\'\n", accepted_command_to_string(support_list->command->name)); //TO BE CHANGED
			fprintf (stderr, "Non \'property\' children are allowed only in control tag of type \'if\',\'while\' and \'repeat\'");
			return -1;
	}
	
	// set command name as end of current tag
	command_end.name = support_list->command->name+1;
	command_end.depth = support_list->command->depth;

	// insert command end in mission list
	insert_command_in_mission_list (/*mission, */&command_end);
	
	// complete the link between the opening and the closing tag
	support_list->command->option3.cmd_ptr = mission->to_execute;
	
	// prune the head node of the support list
	support_elem = support_list;
	support_list = support_list->next;
	free (support_elem);
	
	return 0;
}

int update_support_list (mission_command_t *command)
{
	mission_initialize_support_t *support_elem = NULL, *p = support_list;

	// should not happen
	if (command == NULL)
	{
		fprintf (stderr, "Can't update the support list - Empty command element\n");
		return -1;
	}

	if (command->name == accepted_command_jump ||
		command->name == accepted_command_repeat ||
		command->name == accepted_command_while ||
		command->name == accepted_command_if)
	{
		//add a node on the head of the list
		support_elem = malloc (sizeof(mission_initialize_support_t));
		if (support_elem == NULL)
		{
			fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
			return -1;
		}
		memset (support_elem, 0, sizeof (mission_initialize_support_t));
	
		support_elem->name = command->name;
		support_elem->command = command;
		support_elem->line_counter = (command->name == accepted_command_jump)? mission_n_children : 0;
		
		if (command->name != accepted_command_jump || support_list == NULL)
		{
			// head insert
			support_elem->next = support_list;
			support_list = support_elem;
		}
		else
		{
			//tail insert
			while (p->next != NULL)
				p = p->next;
				
			p->next = support_elem;
			support_elem->next = NULL;
		}
	}
	
	return 0;
}

int mission_file_parse (xmlNode *current_node, int node_depth)
{
	int child_depth = node_depth + 1;
	mission_command_t command;
	xmlNode *node = NULL;
	
	for (node = current_node->children; node; node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE || xmlNodeIsText(node))
			continue;
		
		memset (&command, 0, sizeof (mission_command_t));
		
		if (!strcmp((const char *) node->name, accepted_tag_to_string(accepted_tag_command)))
		{
			if (process_command_node (node, child_depth, &command) < 0)
			{
				fprintf (stderr, "Unable to parse a command node\n");
				return -1;
			}
		}
		else if (!strcmp((const char *) node->name, accepted_tag_to_string(accepted_tag_control)))
		{
			if (process_control_node (node, child_depth, &command) < 0)
			{
				fprintf (stderr, "Unable to parse a control node\n");
				return -1;
			}
		}
		else if (!strcmp((const char *) node->name, accepted_tag_to_string(accepted_tag_property)))
			continue;	// DO NOTHING - This tag will be parsed by the parent node (and can't have child)
		else
		{
			// ERROR - Invalid tag
			if (!strcmp((const char *) node->name, accepted_tag_to_string(accepted_tag_mission)))
				fprintf (stderr, "Can't have a \'mission\' tag inside another \'mission\' tag\n");
			else
				fprintf (stderr, "Invalid \'%s\' tag\n", node->name);
			return -1;
		}
				
		// Insert the command in the mission command list
		if (insert_command_in_mission_list (/*mission, */&command) < 0)
			return -1;
		
		// Update line counters of all previous node that still needs to be closed
		if (update_support_list (/*support_list, */mission->to_execute))
			return -1;
		
		// Recursion on the child
		if (mission_file_parse (/*mission, */node, child_depth) < 0)
			return -1;
						
		// Take track of node on the support list
		if (insert_command_end_in_mission_list (/*support_list, mission, */command.depth) < 0)
			return -1;
	}

	return 0;
}


int check_mission_integrity ()
{
	mission_initialize_support_t *support_elem = NULL;
	mission_command_t *command = NULL;
	int mission_n_commands = 0, i;


	for (command = mission->command_list; command; command = command->next)
	{
		// check that only if, while and repeat have a non-property child
		if (command->next != NULL &&
			command->depth < command->next->depth &&
			command->name != accepted_command_repeat &&
			command->name != accepted_command_while &&
			command->name != accepted_command_if)
		{
			fprintf (stderr, "Invalid \'%s\' tag\n", accepted_command_to_string(command->name));
			fprintf (stderr, "Non \'property\' children are allowed only in control tag of type \'if\',\'while\' and \'repeat\'\n");
			return -1;
		}

		if (command->name < accepted_command_delay)
			mission_n_commands++;
	}

	// check if the mission has at least a command child
	if (mission_n_commands == 0)
	{
		fprintf (stderr, "Invalid \'mission\' tag\n");
		fprintf (stderr, "The mission must have at least a command child\n");
		return -1;
	}

	// check jump targets
	while (support_list)
	{
		for (command = mission->command_list, i = 1; command; command = command->next, i++)
		{
			if (command->id == (uint8_t) support_list->command->option2)
			{
				support_list->command->option1 = (double) i - support_list->line_counter;
				support_list->command->option3.cmd_ptr = command;
				break;
			}
		}

		// target not found
		if (command == NULL)
		{
			fprintf (stderr, "Invalid \'jump\' tag\n");
			fprintf (stderr, "Target not found - can't find a tag whit id %f\n", support_list->command->option2);
			return -1;
		}

		// remove the used element from the list
		support_elem = support_list;
		support_list = support_list->next;
		free (support_elem);
	}

	return 0;
}
