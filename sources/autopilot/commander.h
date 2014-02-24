/**
 * @file commander.h
 * Main system state machine definition
 */

#ifndef COMMANDER_H_
#define COMMANDER_H_

	#include "../uav_library/common.h"


	/* function prototypes */
	void* commander_thread_main (void* args);

	/* global variables */
	bool_t commander_initialized;

#endif 
