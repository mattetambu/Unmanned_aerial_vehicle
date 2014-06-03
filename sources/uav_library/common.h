// common.h

#ifndef _COMMON_H_
#define _COMMON_H_

	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#include <errno.h>
	#include <math.h>
	#include <unistd.h>
	#include <sys/types.h>


	/* definitions */
	//#define INTERACTIVE
	//#define DEBUG
	//#define BYPASS_OUTPUT_CONTROLS_AND_DO_UAV_MODEL_TEST_DEMO
	#define START_TEST_THREAD	0


	/* macros */
	#define return_enum_string(ENUM_CODE) case ENUM_CODE: return #ENUM_CODE
	#define return_custom_enum_string(ENUM_CODE, STRING) case ENUM_CODE: return STRING

	/* type definitions */
	typedef unsigned char uint8_t;
	typedef short unsigned int uint16_t;
	typedef unsigned int uint32_t;
	typedef unsigned long uint64_t;
	typedef uint32_t bool_t;


	/* global variables */
	extern int _shutdown_all_systems;
	// pthread_mutex_t shutdown_mutex;

#endif
