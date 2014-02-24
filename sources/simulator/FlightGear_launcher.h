// FlightGear_launcher.h

#ifndef _FLIGHTGEAR_LAUNCHER_H
#define _FLIGHTGEAR_LAUNCHER_H
	
	#include <pthread.h>
	#include "../uav_library/common.h"
	

	#define DEFAULT_UDP_INPUT_PORT		8080
	#define DEFAULT_TCP_INPUT_PORT		10010
	#define DEFAULT_UDP_OUTPUT_PORT		8081
	#define DEFAULT_TCP_OUTPUT_PORT		10011
	#define DEFAULT_IP_ADDRESS			"127.0.0.1"
	#define INPUT_FREQUENCY				"40"
	#define OUTPUT_FREQUENCY			"30"

	#define INPUT_PROTOCOL_FILE_NAME	"in_out_protocol-rad"
	#define OUTPUT_PROTOCOL_FILE_NAME	"in_out_protocol-rad"


	#define ENGINE_AUTOSTART
	//#define START_IN_AIR

	#define CYGWIN_FLIGHTGEAR_LANCH_COMMAND			"\"/cygdrive/c/Program Files/FlightGear/bin/Win64/fgfs\" --fg-root=\"C:\\Program Files\\FlightGear\\data\" --fg-scenery=\"C:\\Program Files\\FlightGear\\data\\Scenery\""
	#define UNIX_FLIGHTGEAR_LANCH_COMMAND			"\"/some_path/FlightGear/bin/fgfs\" --fg-root=\"/some_path/FlightGear/data\" --fg-scenery=\"/some_path/FlightGear/data/Scenery\""
	#ifdef ENGINE_AUTOSTART
		#define FLIGHTGEAR_LANCH_GENERAL_OPTIONS		" --language=it --control=keyboard --enable-fuel-freeze --enable-hud --timeofday=noon --enable-auto-coordination --httpd=5500 --prop:/engines/engine/running=true --prop:/engines/engine/starter=true --prop:/engines/engine/rpm=600"
	#else
		#define FLIGHTGEAR_LANCH_GENERAL_OPTIONS		" --language=it --control=keyboard --enable-fuel-freeze --enable-hud --timeofday=noon --enable-auto-coordination --httpd=5500"
	#endif
	//#define FLIGHTGEAR_LANCH_AIRCRAFT_OPTIONS		" --aircraft=c172p --prop:/engines/engine/magnetos=3"
	#define FLIGHTGEAR_LANCH_AIRCRAFT_OPTIONS		" --aircraft=Rascal110-JSBSim"
	#define FLIGHTGEAR_LANCH_AIRPORT_OPTIONS		" --airport=KHAF"
	#ifdef START_IN_AIR
		#define FLIGHTGEAR_LANCH_IN_AIR_OPTIONS		" --in-air --vc=120 --altitude=100"
	#endif

	
	/* function prototypes */
	void* FlightGear_launcher (void* args);
	
	/* global variables */
	extern pthread_t simulator_thread_id;
	extern int launcher_thread_return_value;

#endif
