// socket_io.c

#include "socket_io.h"


int input_socket = 0, output_socket = 0;

int check_address (char* IP_addr)
{
	struct sockaddr_in address;
	memset (&address, 0, sizeof (address));

	return inet_pton (AF_INET, (const char*) IP_addr, &address.sin_addr);
}

int server_address_initialization (struct sockaddr_in* address, int port)
{
	memset (address, 0, sizeof (struct sockaddr_in));
	address->sin_family = AF_INET;
	address->sin_port = htons(port);	
	address->sin_addr.s_addr = htonl(INADDR_ANY);
	
	return 0;
}

int client_address_initialization (struct sockaddr_in* address, int port, char* IP_addr)
{
	struct hostent* host;
	host = (struct hostent*) gethostbyname (IP_addr);
	if (host == NULL)
		return -1;
	
	memset (address, 0, sizeof (struct sockaddr_in));
	address->sin_family = AF_INET;
	address->sin_port = htons(port);	
	address->sin_addr = *((struct in_addr *) host->h_addr);
	
	return 0;
}

int udp_server_socket_initialization (int* sk, int port)
{ // UDP server socket configuration
	struct sockaddr_in address;
	server_address_initialization (&address, port);
		
	*sk = socket (AF_INET, SOCK_DGRAM, 0);
	if (*sk == -1)
	{
		fprintf (stderr, "SOCKET return by an error (errno: %d)\n", errno);
		return -1;
	}
	
	if (bind (*sk, (struct sockaddr*) &address, sizeof (struct sockaddr)) == -1)
	{
		shutdown(*sk, SHUT_RDWR);
		close (*sk);
		fprintf (stderr, "BIND return by an error (errno: %d)\n", errno);
		return -1;
	}

	return 0;
}

int udp_client_socket_initialization (int* sk, int port)
{ // UDP client socket configuration
	*sk = socket (AF_INET, SOCK_DGRAM, 0);
	if (*sk == -1)
	{
		fprintf (stderr, "SOCKET return by an error (errno: %d)\n", errno);
		return -1;
	}
	
	return 0;
}

int tcp_server_socket_initialization (int* sk, int port)
{ // TCP server socket configuration
	int listen_sk, address_lenght = sizeof(struct sockaddr_in);
	struct sockaddr_in server_address, client_address;
	server_address_initialization (&server_address, port);
	
	listen_sk = socket (AF_INET, SOCK_STREAM, 0);
	if (listen_sk == -1)
	{
		fprintf (stderr, "SOCKET return by an error (errno: %d)\n", errno);
		return -1;
	}
	
	if (bind (listen_sk, (struct sockaddr*) &server_address, sizeof(server_address)) == -1)
	{
		fprintf (stderr, "BIND return by an error (errno: %d)\n", errno);
		return -1;
	}

	if (listen(listen_sk, 10 /*backlog queue length*/) == -1)
	{
		fprintf (stderr, "LISTEN return by an error (errno: %d)\n", errno);
		return -1;
	}
	
	memset (&client_address, 0, address_lenght);
	*sk = accept (listen_sk, (struct sockaddr*) &client_address, &address_lenght);
	if (*sk == -1)
	{
		fprintf (stderr, "ACCEPT return by an error (errno: %d)\n", errno);
		return -1;
	}
	
	shutdown(listen_sk, SHUT_RDWR);
	close(listen_sk);
	
	return 0;
}

int tcp_client_socket_initialization (int* sk, struct sockaddr_in* server_address)
{ // TCP client socket configuration
	int return_value;
	
	*sk = socket (AF_INET, SOCK_STREAM, 0);
	if (*sk == -1)
	{
		fprintf (stderr, "SOCKET return by an error (errno: %d)\n", errno);
		return -1;
	}
		
	return_value = connect (*sk, (struct sockaddr*) server_address, sizeof (struct sockaddr));
	if (return_value == -1)
	{
		shutdown(*sk, SHUT_RDWR);
		close (*sk);
		fprintf (stderr, "CONNECT return by an error (errno: %d)\n", errno);
		return -1;
	}
	
	return 0;
}
