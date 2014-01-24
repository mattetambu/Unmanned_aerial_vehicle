// shell_controller.h

#ifndef	_SHELL_CONTROLLER_H_
#define	_SHELL_CONTROLLER_H_

	#include "common.h"
	#include <pthread.h>
	
	/* function prototypes */
	void* shell_controller_loop (void* args);
			
	/* global variables */
	extern sem_t shell_semaphore;
	extern pthread_t shell_thread_id;
	
#endif
