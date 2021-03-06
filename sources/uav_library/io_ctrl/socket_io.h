// socket_io.h
 
#ifndef	_SOCKET_IO_H_
#define	_SOCKET_IO_H_

	#include <netinet/in.h>
	#include "../common.h"

	/* function prototypes */
	int check_address (char* IP_addr);
	int client_address_initialization (struct sockaddr_in* address, int port, char* IP_addr);
	int server_address_initialization (struct sockaddr_in* address, int port);
	int udp_server_socket_initialization (int* sk, int port);
	int udp_client_socket_initialization (int* sk, int port);
	int tcp_server_socket_initialization (int* sk, int port);
	int tcp_client_socket_initialization (int* sk,  struct sockaddr_in* server_address);
	
	/* global variables */
	extern int input_socket, output_socket;

#endif
