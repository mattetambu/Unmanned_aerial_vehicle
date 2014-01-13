// primary_loop.h

#ifndef	_PRIMARY_LOOP_H_
#define	_PRIMARY_LOOP_H_

	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#include <signal.h>
	#include <pthread.h>
	#include "common.h"
	#include "comunicator.h"
	#include "autopilot.h"
	#include "aircraft.h"
	#include "plot.h"
	
	
	#define	INPUT_DATA_MAX_LENGTH		1024
	#define	OUTPUT_DATA_MAX_LENGTH		512

	/* function prototypes */
	int parse_flight_data (double* flight_data, char* received_packet);
	int create_output_packet (char* output_packet, double* flight_controls);
	void* primary_loop (void* args);
	
	/* global variables */
	extern int shutdown_primary_thread;
	extern pthread_mutex_t shutdown_mutex;
	extern int primary_thread_return_value;

#endif
