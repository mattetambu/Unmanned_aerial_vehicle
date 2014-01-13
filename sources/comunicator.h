// comunicator.h
 
#ifndef	_COMUNICATOR_H_
#define	_COMUNICATOR_H_

	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#include "common.h"
	#include "socket_io.h"

	
	/* function prototypes */
	int receive_input_packet (char* received_packet, int data_lenght, int input_socket);
	int send_output_packet (char* output_packet, int data_lenght, int output_socket, struct sockaddr_in* address);
	
	/* global variables */
	extern int input_port, output_port;
	extern char output_address[20];


#endif
