// comunicator_loop.h

#ifndef	_COMUNICATOR_LOOP_H_
#define	_COMUNICATOR_LOOP_H_

	#include <pthread.h>
	#include "../uav_library/common.h"
	
	
	#define	INPUT_DATA_MAX_LENGTH		1024
	#define	OUTPUT_DATA_MAX_LENGTH		512

	/* function prototypes */
	void* comunicator_loop (void* args);
	
	/* global variables */
	extern pthread_t comunicator_thread_id;
	extern int comunicator_thread_return_value;
#ifdef START_AUTOPILOT_WHEN_SIMULATOR_IS_READY
	extern int simulator_ready;
#endif
	
#endif
