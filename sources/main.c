// main.c

#include <getopt.h>
#include "common.h"
#include "socket_io.h"
#include "autopilot_logic.h"
#include "packet_manage.h"
#include "plot.h"
#include "main.h"

#define DEFAULT_UDP_INPUT_PORT		6100
#define DEFAULT_TCP_INPUT_PORT		5100
#define DEFAULT_UDP_OUTPUT_PORT		6000
#define DEFAULT_TCP_OUTPUT_PORT		5000
#define DEFAULT_IP_ADDRESS			"127.0.0.1"
#define	INPUT_DATA_MAX_LENGTH	1024
#define	OUTPUT_DATA_MAX_LENGTH	512

	
int input_port = -1;
int output_port = -1;
char* output_address = NULL;

float flight_data[N_USED_PROPERTIES];
float flight_controls[N_USED_CONTROLS];


void usage ()
{
	printf ("USAGE:\n");
	printf ("   FlightGear_comunicator <options>\n");
	printf ("OPTIONS:\n");
	printf ("   -i --input-port\n");
	printf ("\t Set the port where to receive flight-data from FlightGear\n");
	printf ("   -o --output-port\n");
	printf ("\t Set the port where to send the flight-controls for FlightGear\n");
	printf ("   -a --address\n");
	printf ("\t Set the IPv4 address where to send the flight-controls for FlightGear\n");
	printf ("   -t --tcp\n");
	printf ("\t Use TCP connection\n");
	printf ("   -u --udp\n");
	printf ("\t Use UDP connection [DEFAULT]\n");
	printf ("   -d --do-not-send-command\n");
	printf ("\t Don't send flight-controls to FlightGear\n");
	printf ("   -v --verbose\n");
	printf ("\t Display more messages\n");
	printf ("   -h --help\n");
	printf ("\t Show this help\n");
	printf ("\n");
	fflush (stdout);
}

int check_arguments (int argc, char **argv)
{
	int option_index = 0, option_character;
	
	static struct option long_options[] = {
		{"input-port",				required_argument,	0,	'i'},
		{"output-port",				required_argument,	0,	'o'},
		{"address",					required_argument,	0,	'a'},
		{"tcp",						no_argument,		0,	't'},
		{"udp",						no_argument,		0,	'u'},
		{"do-not-send-controls",	no_argument,		0,	'd'},
		{"verbose",					no_argument,		0,	'v'},
		{"help",					no_argument,		0,	'h'}
	};
 
	while ((option_character = getopt_long (argc, argv, "i:o:a:tudvh", long_options, &option_index)) != -1)
	{
		switch (option_character)
		{
			case 'i': //set input_port
				input_port = atoi (optarg);
				break;
				
			case 'o': //set output_port
				output_port = atoi (optarg);
				break;
				
			case 'a': //set output_address
				output_address = strdup (optarg);
				break;

			case 't': //set tcp_flag
				setenv ("TCP_FLAG", "", 0);
				break;
			
			case 'u': //set udp_flag
				setenv ("UDP_FLAG", "", 0);
				break;
			
			case 'd': //set udp_flag
				setenv ("DO_NOT_SEND_CONTROLS", "", 0);
				break;
			
			case 'v': //set verbose_flag
				setenv ("VERBOSE", "", 0);
				break;
				
			case 'h': //help
				usage();
				exit(0);

			case '?': default: // getopt_long already printed an error message
				return -1;
		}
	}
	
	if (optind < argc) { //output of any remaining command line arguments (not options)
		fprintf (stderr, "Not-option elements: ");
		while (optind < argc) fprintf (stderr, "%s ", argv[optind++]);
		fprintf (stderr, "\n");
		return -1;
	}	
	
	if (getenv("TCP_FLAG"))
	{
		if (getenv("UDP_FLAG"))
		{
			fprintf (stderr, "Options -u and -t cannot be used together\n");
			return -1;
		}
	}
	else if (!getenv("UDP_FLAG")) //UDP set as the default protocol
		setenv ("UDP_FLAG", "", 1);
	
	if (input_port == -1)
		input_port = (getenv("TCP_FLAG"))? DEFAULT_TCP_INPUT_PORT : DEFAULT_UDP_INPUT_PORT;
	if (input_port < 1023 ||  input_port > 65535)
		return -1;
	
	if (output_port == -1)
		output_port = (getenv("TCP_FLAG"))? DEFAULT_TCP_OUTPUT_PORT : DEFAULT_UDP_OUTPUT_PORT;
	if (output_port < 1023 ||  output_port > 65535 || output_port == input_port)
		return -1;
	
	if (output_address == NULL)
	{
		output_address = (char*) malloc (20);
		if (output_address == NULL)
		{
			fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
			return -1;
		}
		output_address = DEFAULT_IP_ADDRESS;
	}
	if (!check_address (output_address))
		return -1;
	
	if (getenv("VERBOSE"))
	{
		fprintf (stdout, "Input-port (flight-data): %d\n", input_port);
		if (!getenv("DO_NOT_SEND_CONTROLS"))
		{
			fprintf (stdout, "Output-port (flight-controls): %d\n", output_port);
			fprintf (stdout, "Output-IPv4-address (flight-controls): %s\n", output_address);
		}
		else
			fprintf (stdout, "No flight-control will be sent to FlightGear\n");
		if (getenv("TCP_FLAG"))
			fprintf (stdout, "Using protocol: TCP\n");
		else
			fprintf (stdout, "Using protocol: UDP\n");
		fflush (stdout);
	}
	
	return 0;
}

int main (int n_args, char** args)
{
	int input_socket;
	char received_data[INPUT_DATA_MAX_LENGTH];
	int output_socket;
	char data_sent[OUTPUT_DATA_MAX_LENGTH];
	struct sockaddr_in output_sockaddr;
	
	if (check_arguments (n_args, args) < 0)
	{
		fprintf (stderr, "Bad args - Can't run the program\n");
		usage();
		exit(-1);
	}
		
	if (!getenv("DO_NOT_SEND_CONTROLS") && address_initialization (&output_sockaddr, output_port, output_address) < 0)
	{
		fprintf (stderr, "Can't find the host on the given IP (errno: %d)\n", errno);
		exit(-1);
	}
	
	fprintf (stdout, "\nTrying to establish a connection with FlightGear\n");
	fflush(stdout);
	
	if (getenv("TCP_FLAG"))
	{ //TCP
		fprintf (stdout, "Waiting for FlightGear on port %d\n\n", input_port);
		fflush(stdout);
		
		if (tcp_server_socket_initialization (&input_socket, input_port) < 0)
		{
			fprintf (stderr, "Can't create the input socket correctly (errno: %d)\n", errno);
			exit(-1);
		}
		if (!getenv("DO_NOT_SEND_CONTROLS") && tcp_client_socket_initialization (&output_socket, &output_sockaddr) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly (errno: %d)\n", errno);
			exit(-1);
		}
	}
	else
	{ //UDP
		if (udp_socket_initialization (&input_socket, input_port) < 0)
		{
			fprintf (stderr, "Can't create the input socket correctly (errno: %d)\n", errno);
			exit(-1);
		}
		if (!getenv("DO_NOT_SEND_CONTROLS") && udp_socket_initialization (&output_socket, output_port) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly (errno: %d)\n", errno);
			exit(-1);
		}
		
		fprintf (stdout, "Waiting for FlightGear on port %d\n\n", input_port);
		fflush(stdout);
	}
	
	while (1)
	{
		// INPUTS
		if (receive_input_packet (received_data, INPUT_DATA_MAX_LENGTH, input_socket) <= 0)
		{
			fprintf (stderr, "Can't receive the flight-data from FlightGear (errno: %d)\n", errno);
			exit(-1);
		}
		
		if (parse_flight_data (flight_data, received_data) < N_USED_PROPERTIES)
		{
			fprintf (stderr, "Can't parse the flight-data\n");
			exit(-1);
		}
		
		if (plot_flight_data (flight_data) < 0)
		{
			fprintf (stderr, "Can't plot the flight-data\n");
			exit(-1);
		}
		
		if (!getenv("DO_NOT_SEND_CONTROLS"))
		{
			// AUTOPILOT LOGIC
			if (compute_flight_controls (flight_controls, flight_data) < 0)
			{
				fprintf (stderr, "Can't compute flight controls\n");
				exit(-1);
			}
			
			
			// OUTPUTS
			if (plot_flight_controls (flight_controls) < 0)
			{
				fprintf (stderr, "Can't plot the flight-controls\n");
				exit(-1);
			}
			
			if (create_output_packet (data_sent, flight_controls) < N_USED_CONTROLS)
			{
				fprintf (stderr, "Can't create the packet to be sent to FlightGear\n");
				exit(-1);
			}
			
#ifdef DEBUG
			fprintf (stdout, "Output packet created and ready to be sent: %s\n", data_sent);
			fflush (stdout);
#endif
			
			if (send_output_packet (data_sent, strlen(data_sent), output_socket, &output_sockaddr) < strlen(data_sent))
			{
				fprintf (stderr, "Can't send the packet to FlightGear (errno: %d)\n", errno);
				exit(-1);
			}
		}
	}
	
	return 0;
}