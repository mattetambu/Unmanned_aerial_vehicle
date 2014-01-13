// FlightGear_launcher.h

#ifndef _FLIGHTGEAR_LAUNCHER_H
#define _FLIGHTGEAR_LAUNCHER_H
	
	#include <signal.h>
	#include <pthread.h>
	#include "common.h"
	#include "comunicator.h"
	
	#define DEFAULT_UDP_INPUT_PORT		8080
	#define DEFAULT_TCP_INPUT_PORT		10010
	#define DEFAULT_UDP_OUTPUT_PORT		8081
	#define DEFAULT_TCP_OUTPUT_PORT		10011
	#define DEFAULT_IP_ADDRESS			"127.0.0.1"
	#define INPUT_FREQUENCY				"40"
	#define OUTPUT_FREQUENCY			"30"

	#ifdef USE_RADIANS
		#define INPUT_PROTOCOL_FILE_NAME	"in_out_protocol-rad"
		#define OUTPUT_PROTOCOL_FILE_NAME	"in_out_protocol-rad"
	#else
		#define INPUT_PROTOCOL_FILE_NAME	"in_out_protocol-deg"
		#define OUTPUT_PROTOCOL_FILE_NAME	"in_out_protocol-deg"
	#endif

	#define WINDOWS_FLIGHTGEAR_LANCH_COMMAND	"\"C:\\Program Files\\FlightGear\\bin\\Win64\\fgfs\" --fg-root=\"C:\\Program Files\\FlightGear\\data\" --fg-scenery=\"C:\\Program Files\\FlightGear\\data\\Scenery\""
	#define CYGWIN_FLIGHTGEAR_LANCH_COMMAND		"\"/cygdrive/c/Program Files/FlightGear/bin/Win64/fgfs\" --fg-root=\"C:\\Program Files\\FlightGear\\data\" --fg-scenery=\"C:\\Program Files\\FlightGear\\data\\Scenery\""
	#define UNIX_FLIGHTGEAR_LANCH_COMMAND		"\"/some_path/FlightGear/bin/fgfs\" --fg-root=\"/some_path/FlightGear/data\" --fg-scenery=\"/some_path/FlightGear/data/Scenery\""
	#define FLIGHTGEAR_LANCH_OPTIONS			" --language=it --control=keyboard --units-meters --enable-fuel-freeze --aircraft=c172p --airport=KHAF --enable-hud --timeofday=noon --in-air --altitude=1000 --vc=120 --prop:/engines/engine/running=true --prop:/engines/engine/rpm=1500 --enable-auto-coordination --httpd=5500"
	//--aircraft=Rascal110-JSBSim --altitude=100 --vc=120
	//--aircraft=c172p --altitude=1000

	
	/* function prototypes */
	void* FlightGear_launcher (void* args);
	
	/* global variables */
	extern int launcher_thread_return_value;

#endif
