// test_thread.h

#ifndef	_TEST_THREAD_H_
#define	_TEST_THREAD_H_

	#include <pthread.h>
	#include "uav_library/common.h"
	
	#define	INPUT_DATA_MAX_LENGTH		1024
	#define	OUTPUT_DATA_MAX_LENGTH		512

	/* function prototypes */
	void* test_thread_body (void* args);
	void* test_thread_body1 (void* args);
	void* test_thread_body2 (void* args);
	
	/* global variables */
	extern pthread_t test_thread_id, test_thread_id2;
	
#endif
