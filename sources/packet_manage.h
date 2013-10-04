// packet_manage.h

#ifndef	_PACKET_MANAGE_H_
#define	_PACKET_MANAGE_H_

	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>

	/* function prototypes */
	int receive_input_packet (char* received_packet, int data_lenght, int input_socket);
	int parse_flight_data (float* FDM_data, char* received_packet);
	int create_output_packet (char* output_packet, float* controls);
	int send_output_packet (char* output_packet, int data_lenght, int output_socket, struct sockaddr_in* address);
	
	/* global variables */
	//empty

#endif
