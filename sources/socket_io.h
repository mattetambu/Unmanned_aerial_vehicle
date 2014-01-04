// socket_io.h
 
#ifndef	_SOCKET_IO_H_
#define	_SOCKET_IO_H_

	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>

	/* function prototypes */
	int check_address (char* IP_addr);
	int client_address_initialization (struct sockaddr_in* address, int port, char* IP_addr);
	int server_address_initialization (struct sockaddr_in* address, int port);
	int udp_server_socket_initialization (int* sk, int port);
	int udp_client_socket_initialization (int* sk, int port);
	int tcp_server_socket_initialization (int* sk, int port);
	int tcp_client_socket_initialization (int* sk,  struct sockaddr_in* server_address);
	int receive_input_packet (char* received_packet, int data_lenght, int input_socket);
	int send_output_packet (char* output_packet, int data_lenght, int output_socket, struct sockaddr_in* address);
	
	/* global variables */
	extern int input_socket;
	extern int output_socket;

#endif
