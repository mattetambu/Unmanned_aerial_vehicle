// console_controller.h

#ifndef	_CONSOLE_CONTROLLER_H_
#define	_CONSOLE_CONTROLLER_H_

	#include <pthread.h>
	#include "../common.h"
	
	/* function prototypes */
	int get_console_unique_control ();
	void release_console_control ();		
	void* console_controller_loop (void* args);
	
	/* global variables */
	extern pthread_t console_thread_id;
	
#endif
