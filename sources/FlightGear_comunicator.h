// FlightGear_comunicator.h

#ifndef	_FLIGHTGEAR_COMUNICATOR_H_
#define	_FLIGHTGEAR_COMUNICATOR_H_

	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	
	#define	INPUT_DATA_MAX_LENGTH		1024
	#define	OUTPUT_DATA_MAX_LENGTH		512

	/* function prototypes */
	int parse_flight_data (double* flight_data, char* received_packet);
	int create_output_packet (char* output_packet, double* flight_controls);
	void* FlightGear_comunicator (void* args);
	
	/* global variables */
	extern int input_port;
	extern int output_port;
	extern char output_address[20];
	extern int comunicator_thread_return_value;
	extern int SHUTDOWN_COMUNICATOR;
	extern pthread_mutex_t mutex;

#endif
