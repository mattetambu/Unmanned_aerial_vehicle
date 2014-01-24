// common.h

#ifndef _COMMON_H_
#define _COMMON_H_

	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#include <errno.h>
	#include <math.h>
	#include <sys/types.h>
	#include <unistd.h>

	#define DEBUG
	
	/**< xxx to be removed */
	//#define DO_NOT_COMUNICATE
	//#define INTERACTIVE


	#ifndef NULL
		#define NULL	((void*) 0)
	#endif

	#ifndef size_t
		#define size_t	unsigned int
	#endif

	#define N_SPACES_PER_TAB	5


	#define return_enum_string(ENUM_CODE) case ENUM_CODE: return #ENUM_CODE
	#define return_custom_enum_string(ENUM_CODE, STRING) case ENUM_CODE: return STRING
	
	#define LENGTH_MISURE_UNIT	"meters"
	#define ANGLE_MISURE_UNIT	"degrees"
	#define TIME_MISURE_UNIT	"seconds"
	#define METERS_TO_FEETS_FACTOR 3.2808399
	
	typedef unsigned char uint8_t;
	typedef short unsigned int uint16_t;
	typedef unsigned int uint32_t;
	typedef unsigned long uint64_t;
	
	typedef struct map_point_t
	{
		float altitude;
		double latitude, longitude;
	} map_point_t;
	

	/* global variables */
	// pthread_mutex_t shutdown_mutex;
	extern int _shutdown_all_systems;

#endif
