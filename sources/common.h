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

	#define min(x1, x2) ((x1 < x2)? x1 : x2)
	#define max(x1, x2) ((x1 > x2)? x1 : x2)
	
	//#define INTERACTIVE
	#define DEBUG
	
	
	// FDM (AIRCRAFT SETTINGS)
	//	ORIENTATION
	#define ROLL			0
	#define PITCH			1
	#define HEADING			2
	#define ROLL_RATE		3
	#define PITCH_RATE		4
	#define YAW_RATE		5
	//	VELOCITIES
	#define AIRSPEED		6
	#define U_BODY			7
	#define V_BODY			8
	#define W_BODY			9
	//	ACCELERATION
	#define NLF				10
	#define X_ACCEL			11
	#define Y_ACCEL			12
	#define Z_ACCEL			13

	// CONTROLS
	//	FLIGHT CONTROLS
	#define AILERON			0
	#define ELEVATOR		1
	#define RUDDER			2
	
	#define N_USED_PROPERTIES	14
	#define N_USED_CONTROLS		3

	typedef unsigned char uint8_t;
	typedef short unsigned int uint16_t;
	typedef unsigned int uint32_t;
	typedef unsigned long uint64_t;

#endif
