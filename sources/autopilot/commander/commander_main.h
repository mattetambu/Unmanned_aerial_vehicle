/**
 * @file commander_main.h
 * Main system state machine definition
 */

#ifndef COMMANDER_MAIN_H_
#define COMMANDER_MAIN_H_

	#include "../../uav_library/common.h"


	/* function prototypes */
	int commander_params_define ();
	void* commander_thread_main (void* args);

	/* global variables */
	extern bool_t commander_initialized;

#endif 
