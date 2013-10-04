// socket_io.c

#include "common.h"
#include "socket_io.h"

int check_address (char* IP_addr)
{
	struct sockaddr_in address;
	memset (&address, 0, sizeof (address));

	return inet_pton (AF_INET, (const char*) IP_addr, &address.sin_addr);
}

int address_initialization (struct sockaddr_in* address, int port, char* IP_addr /*NULL for INADDR_ANY*/)
{ // Client socket configuration
	struct hostent* host;
	memset (address, 0, sizeof (struct sockaddr_in));
	
	(*address).sin_family = AF_INET;
	(*address).sin_port = htons(port);	
	if (IP_addr == NULL)
		(*address).sin_addr.s_addr = INADDR_ANY;
	else
	{
		host = (struct hostent*) gethostbyname (IP_addr);
		if (host == NULL)
			return -1;
		(*address).sin_addr = *((struct in_addr *) host->h_addr);
	}

	return 0;
}

int udp_socket_initialization (int* sk, int port)
{ // UDP socket configuration
	struct sockaddr_in address;
	
	if (address_initialization (&address, port, NULL) < 0)
	{
		fprintf (stderr, "Can't find the host on the given IP\n");
		return -1;
	}
	
	*sk = socket (PF_INET, SOCK_DGRAM, 0);
	if (*sk == -1 ||
		bind (*sk, (struct sockaddr*) &address, sizeof (address)) == -1)
		return -1;

	return 0;
}

int tcp_server_socket_initialization (int* sk, int port)
{ // TCP server socket configuration
	int listen_sk, address_lenght = sizeof(struct sockaddr_in);
	struct sockaddr_in server_address, client_address;
	
	if (address_initialization (&server_address, port, NULL) < 0)
	{
		fprintf (stderr, "Can't find the host on the given IP\n");
		return -1;
	}
	
	listen_sk = socket (PF_INET, SOCK_STREAM, 0);
	if (listen_sk == -1 ||
		bind (listen_sk, (struct sockaddr*) &server_address, sizeof(server_address)) == -1 ||
		listen(listen_sk, 10 /*backlog queue length*/) == -1)
		return -1;
	
	memset (&client_address, 0, address_lenght);
	*sk = accept (listen_sk, (struct sockaddr*) &client_address, &address_lenght);
	if (*sk == -1)
		return -1;
	
	shutdown(listen_sk, SHUT_RDWR);
	close(listen_sk);
	
	return 0;
}

int tcp_client_socket_initialization (int* sk, struct sockaddr_in* server_address)
{ // TCP client socket configuration
	*sk = socket (PF_INET, SOCK_STREAM, 0);
	if (*sk == -1)
		return -1;
		
	return connect (*sk, (struct sockaddr*) server_address, sizeof (struct sockaddr_in));
}
