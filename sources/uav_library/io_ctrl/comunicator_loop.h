// comunicator_loop.h

#ifndef	_COMUNICATOR_LOOP_H_
#define	_COMUNICATOR_LOOP_H_

	#include <pthread.h>
	#include "../common.h"
	
	
	#define	INPUT_DATA_MAX_LENGTH		1024
	#define	OUTPUT_DATA_MAX_LENGTH		512

	/* function prototypes */
	void* comunicator_loop (void* args);
	
	/* global variables */
	extern pthread_t primary_thread_id;
	extern int primary_thread_return_value;
	
#endif
