// autopilot.h

#ifndef	_AUTOPILOT_H_
#define	_AUTOPILOT_H_

	#include "../uav_library/common.h"
	

	/* function prototypes */
	void* autopilot_loop (void* args);
	
	/* global variables */
	extern pthread_t autopilot_thread_id;
	extern int autopilot_thread_return_value;
	
#endif
