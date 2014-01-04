// mission_file_parser.h

#ifndef	_MISSION_FILE_PARSER_H_
#define	_MISSION_FILE_PARSER_H_

	#include <libxml/parser.h>
	#include "common.h"
	#include "mission_logic.h"
	#include "mission_parameters.h"
	
	/* function prototypes */
	int process_mission_node (xmlNode *node);
	int mission_file_parse (xmlNode *current_node, int node_depth);
	int check_mission_integrity ();
	
	/* global variables */
	
	
#endif
