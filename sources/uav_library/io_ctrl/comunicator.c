// comunicator.c

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "comunicator.h"
#include "../common.h"

int input_port = 0, output_port = 0;
char output_address[20];

int receive_input_packet (char* received_packet, int data_lenght, int input_socket)
{
	struct sockaddr_in address;
	int bytes_read, address_length = sizeof (struct sockaddr);
	
	if (getenv("TCP_FLAG"))
		bytes_read = recv (input_socket, received_packet, data_lenght, 0);
	else
	{ //Wait for new UDP packet, and read it into received_packet
		memset (&address, 0, address_length);
		bytes_read = recvfrom (input_socket, received_packet, data_lenght, 0, (struct sockaddr*) &address, &address_length);
	}
	
	if (bytes_read > 0)
	{
		received_packet[bytes_read] = '\0';
		/*if (getenv("VERBOSE"))
		{ //Print the raw data
			fprintf (stdout, "\nReceived packet:\n%s", received_packet);
			fflush (stdout);
		}*/
	}
	
	return bytes_read;
}

int send_output_packet (char* output_packet, int data_lenght, int output_socket, struct sockaddr_in* address)
{
	int bytes_sent;
	
	if (getenv("TCP_FLAG"))
		bytes_sent = send (output_socket, output_packet, data_lenght, 0);
	else
		bytes_sent = sendto (output_socket, output_packet, data_lenght, 0, (struct sockaddr*) address, sizeof (struct sockaddr));
	
	/*if (bytes_sent == data_lenght && getenv("VERBOSE"))
	{ //Print Control inputs to stdout
		fprintf (stdout, "\nControl signals sent\n%s\n", output_packet);
		fflush (stdout);  		
	}*/
	
	return bytes_sent;
}
