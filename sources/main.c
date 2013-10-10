// main.c

#include <getopt.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include "common.h"
#include "socket_io.h"
#include "autopilot_logic.h"
#include "packet_manage.h"
#include "plot.h"
#include "GUI.h"
#include "main.h"

#define	INPUT_DATA_MAX_LENGTH		1024
#define	OUTPUT_DATA_MAX_LENGTH		512

#define DEFAULT_UDP_INPUT_PORT		8080
#define DEFAULT_TCP_INPUT_PORT		10010
#define DEFAULT_UDP_OUTPUT_PORT		8081
#define DEFAULT_TCP_OUTPUT_PORT		10011
#define DEFAULT_IP_ADDRESS			"127.0.0.1"
#define INPUT_FREQUENCY				"30"
#define OUTPUT_FREQUENCY			"30"
#define INPUT_PROTOCOL_FILE_NAME	"input_protocol"
#define OUTPUT_PROTOCOL_FILE_NAME	"output_protocol"

#define WINDOWS_FLIGHTGEAR_LANCH_COMMAND	"\"C:\\Program Files\\FlightGear\\bin\\Win64\\fgfs\" --fg-root=\"C:\\Program Files\\FlightGear\\data\" --fg-scenery=\"C:\\Program Files\\FlightGear\\data\\Scenery\""
#define CYGWIN_FLIGHTGEAR_LANCH_COMMAND		"\"/cygdrive/c/Program Files/FlightGear/bin/Win64/fgfs\" --fg-root=\"C:\\Program Files\\FlightGear\\data\" --fg-scenery=\"C:\\Program Files\\FlightGear\\data\\Scenery\""
#define UNIX_FLIGHTGEAR_LANCH_COMMAND		"\"/some_path/FlightGear/bin/fgfs\" --fg-root=\"/some_path/FlightGear/data\" --fg-scenery=\"/some_path/FlightGear/data/Scenery\""
#define FLIGHTGEAR_LANCH_OPTIONS			" --language=it --control=keyboard --units-meters --enable-fuel-freeze --aircraft=c172p --airport=KHAF --enable-hud --timeofday=noon --in-air --altitude=2000 --vc=120 --enable-auto-coordination --httpd=5500"


int input_port = -1;
int output_port = -1;
char output_address[20];

float flight_data[N_USED_PROPERTIES];
float flight_controls[N_USED_CONTROLS];

int launcher_thread_return_value, comunicator_thread_return_value;
int input_socket, output_socket;
pthread_mutex_t mutex;
int SHUTDOWN_COMUNICATOR = 0;

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
	printf ("   -g --gui\n");
	printf ("\t Start the GUI\n");
	printf ("   -s --start-FlightGear\n");
	printf ("\t Start FlightGear whit the correct options\n");
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
		{"gui",						no_argument,		0,	'g'},
		{"start-FlightGear",		no_argument,		0,	's'},
		{"do-not-send-controls",	no_argument,		0,	'd'},
		{"verbose",					no_argument,		0,	'v'},
		{"help",					no_argument,		0,	'h'}
	};
 
	while ((option_character = getopt_long (argc, argv, "i:o:a:tugsdvh", long_options, &option_index)) != -1)
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
				strcpy (output_address, (char*) optarg);
				break;

			case 't': //set tcp_flag
				setenv ("TCP_FLAG", "", 0);
				break;
			
			case 'u': //set udp_flag
				setenv ("UDP_FLAG", "", 0);
				break;
			
			case 'g': //set start_flightgear_flag
				setenv ("START_GUI", "", 0);
				break;

			case 's': //set start_flightgear_flag
							setenv ("START_FLIGHTGEAR", "", 0);
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
	
	if (!check_address (output_address))
	strcpy (output_address, DEFAULT_IP_ADDRESS);
	
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

void* FlightGear_launcher (void* arg)
{
	int command_length = 0;
	char* command;
	char c_input_port[5], c_output_port[5];
	
	sprintf(c_input_port, "%d", input_port);
	sprintf(c_output_port, "%d", output_port);
	
	sleep(1);
	
#if defined(__CYGWIN__) || defined(CYGWIN) //CYGWIN
	command_length += strlen ((char*) CYGWIN_FLIGHTGEAR_LANCH_COMMAND);
#elif defined(__WINDOWS__) || defined(WINDOWS) || defined(windows) || defined(G_OS_WIN32) || defined(G_PLATFORM_WIN32) //WINDOWS
	command_length += strlen ((char*) WINDOWS_FLIGHTGEAR_LANCH_COMMAND);
#else //defined(__LINUX__) || defined(LINUX) || defined(linux) || defined(G_OS_UNIX) //UNIX
	command_length += strlen ((char*) UNIX_FLIGHTGEAR_LANCH_COMMAND);
#endif

	command_length += strlen ((char*) FLIGHTGEAR_LANCH_OPTIONS);
	command_length += strlen ((char*) " --generic=socket,out,")
					+ strlen ((char*) INPUT_FREQUENCY)
					+ strlen ((char*) ",127.0.0.1,")
					+ strlen ((char*) c_input_port)
					+ strlen ((char*) ",***,output_protocol"); // *** means tcp or udp

	if (!getenv("DO_NOT_SEND_CONTROLS"))
	{
		command_length += strlen ((char*) " --generic=socket,in,")
						+ strlen ((char*) OUTPUT_FREQUENCY)
						+ strlen ((char*) ",127.0.0.1,")
						+ strlen ((char*) c_input_port)
						+ strlen ((char*) ",***,input_protocol"); // *** means tcp or udp	
	}
	
	command = malloc (command_length+1);
	if (command == NULL)
	{
		fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
		launcher_thread_return_value = -1;
		pthread_exit(&launcher_thread_return_value);
	}

#if defined(__CYGWIN__) || defined(CYGWIN) //CYGWIN
	strcpy (command, (char*) CYGWIN_FLIGHTGEAR_LANCH_COMMAND);
#elif defined(__WINDOWS__) || defined(WINDOWS) || defined(windows) || defined(G_OS_WIN32) || defined(G_PLATFORM_WIN32) //WINDOWS
	strcpy (command, (char*) WINDOWS_FLIGHTGEAR_LANCH_COMMAND);
#else //defined(__LINUX__) || defined(LINUX) || defined(linux) || defined(G_OS_UNIX) //UNIX
	strcpy (command, (char*) UNIX_FLIGHTGEAR_LANCH_COMMAND);
#endif

	strcat (command, (char*) FLIGHTGEAR_LANCH_OPTIONS);
	strcat (command, (char*) " --generic=socket,out,");
	strcat (command, (char*) INPUT_FREQUENCY);
	strcat (command, (char*) ",127.0.0.1,");
	strcat (command, (char*) c_input_port);
	strcat (command, (char*) ",");
	if (getenv("TCP_FLAG"))
		strcat (command, (char*) "tcp");
	else
		strcat (command, (char*) "udp");
	strcat (command, (char*) ",output_protocol");

	if (!getenv("DO_NOT_SEND_CONTROLS"))
	{
		strcat (command, (char*) " --generic=socket,in,");
		strcat (command, (char*) OUTPUT_FREQUENCY);
		strcat (command, (char*) ",127.0.0.1,");
		strcat (command, (char*) c_output_port);
		strcat (command, (char*) ",");
		if (getenv("TCP_FLAG"))
			strcat (command, (char*) "tcp");
		else
			strcat (command, (char*) "udp");
		strcat (command, (char*) ",input_protocol");
	}

	fprintf (stdout, "\nTrying to launch FlightGear ");
	if (getenv("VERBOSE"))
		fprintf (stdout, "using the command:\n%s\n", command);
	fprintf (stdout, "\n");
	fflush(stdout);
	
	launcher_thread_return_value = system (command);
	free (command);
	
	pthread_exit(&launcher_thread_return_value);
	return 0;
}

void* FlightGear_comunicator (void* arg)
{
	char received_data[INPUT_DATA_MAX_LENGTH];
	char data_sent[OUTPUT_DATA_MAX_LENGTH];
	struct sockaddr_in output_sockaddr;
		
	if (!getenv("DO_NOT_SEND_CONTROLS") && client_address_initialization (&output_sockaddr, output_port, output_address) < 0)
	{
		fprintf (stderr, "Can't find the host on the given IP\n");
		comunicator_thread_return_value = -1;
		pthread_exit(&comunicator_thread_return_value);
	}
	
	fprintf (stdout, "\nTrying to establish a connection with FlightGear\n");
	fflush(stdout);
	
	if (getenv("TCP_FLAG"))
	{ //TCP
		fprintf (stdout, "Waiting for FlightGear on port %d\n\n", input_port);
		fflush(stdout);
		
		if (tcp_server_socket_initialization (&input_socket, input_port) < 0)
		{
			fprintf (stderr, "Can't create the input socket correctly\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
		if (!getenv("DO_NOT_SEND_CONTROLS") && tcp_client_socket_initialization (&output_socket, &output_sockaddr) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
	}
	else
	{ //UDP
		if (udp_server_socket_initialization (&input_socket, input_port) < 0)
		{
			fprintf (stderr, "Can't create the input socket correctly\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
		
		if (!getenv("DO_NOT_SEND_CONTROLS") && udp_client_socket_initialization (&output_socket, output_port) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}

		fprintf (stdout, "Waiting for FlightGear on port %d\n\n", input_port);
		fflush(stdout);
	}
	
	sleep(3);
	
	while (1)
	{
		// INPUTS
		if (receive_input_packet (received_data, INPUT_DATA_MAX_LENGTH, input_socket) <= 0)
		{
			sleep(1);
			pthread_mutex_lock (&mutex);
			if (!SHUTDOWN_COMUNICATOR)
			{
				fprintf (stderr, "Can't receive the flight-data from FlightGear\n");
				comunicator_thread_return_value = -1;
			}
			pthread_mutex_unlock (&mutex);
			pthread_exit(&comunicator_thread_return_value);
		}
		
		if (parse_flight_data (flight_data, received_data) < N_USED_PROPERTIES)
		{
			fprintf (stderr, "Can't parse the flight-data\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
		
		if (plot_flight_data (flight_data) < 0)
		{
			fprintf (stderr, "Can't plot the flight-data\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
		
		if (!getenv("DO_NOT_SEND_CONTROLS"))
		{
			// AUTOPILOT LOGIC
			if (compute_flight_controls (flight_controls, flight_data) < 0)
			{
				fprintf (stderr, "Can't compute flight controls\n");
				comunicator_thread_return_value = -1;
				pthread_exit(&comunicator_thread_return_value);
			}
						
			// OUTPUTS
			if (plot_flight_controls (flight_controls) < 0)
			{
				fprintf (stderr, "Can't plot the flight-controls\n");
				comunicator_thread_return_value = -1;
				pthread_exit(&comunicator_thread_return_value);
			}
			
			if (create_output_packet (data_sent, flight_controls) < N_USED_CONTROLS)
			{
				fprintf (stderr, "Can't create the packet to be sent to FlightGear\n");
				comunicator_thread_return_value = -1;
				pthread_exit(&comunicator_thread_return_value);
			}
			
			if (send_output_packet (data_sent, strlen(data_sent), output_socket, &output_sockaddr) < strlen(data_sent))
			{
				sleep(1);
				pthread_mutex_lock (&mutex);
				if (!SHUTDOWN_COMUNICATOR)
				{
					fprintf (stderr, "Can't send the packet to FlightGear\n");
					comunicator_thread_return_value = -1;
				}
				pthread_mutex_unlock (&mutex);
				pthread_exit(&comunicator_thread_return_value);
			}
		}
	}
	
	return 0;
}


int main (int n_args, char** args)
{
	int* return_values[2];
	pthread_t launcher_thread_ID, comunicator_thread_ID, GUI_thread_ID;
	pthread_mutex_init (&mutex, NULL);
	
	if (check_arguments (n_args, args) < 0)
	{
		fprintf (stderr, "Bad args - Can't run the program\n");
		usage();
		exit(-1);
	}


	if (getenv("START_FLIGHTGEAR") && pthread_create (&launcher_thread_ID, NULL, FlightGear_launcher, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to launch FlightGear (errno: %d)\n", errno);
		exit(-1);
	}
		
	if (pthread_create (&comunicator_thread_ID, NULL, FlightGear_comunicator, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to comunicate whit FlightGear (errno: %d)\n", errno);
		exit(-1);
	}
	
	if (getenv("START_GUI") &&  pthread_create (&GUI_thread_ID, NULL, start_GUI, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to start the GUI (errno: %d)\n", errno);
		exit(-1);
	}


	if (getenv("START_FLIGHTGEAR"))
			pthread_join(launcher_thread_ID, (void**)&(return_values[0]));
	
	pthread_mutex_lock (&mutex);
	SHUTDOWN_COMUNICATOR = 1;
	pthread_mutex_unlock (&mutex);
	
	shutdown(input_socket, SHUT_RDWR);
	close(input_socket);
	if (!getenv("DO_NOT_SEND_CONTROLS"))
	{
		shutdown(output_socket, SHUT_RDWR);
		close(output_socket);
	}
	pthread_join(comunicator_thread_ID, (void**)&(return_values[1]));
	pthread_mutex_destroy (&mutex);
	
	if (getenv("VERBOSE"))
	{
		fprintf (stdout, "\nFlightGear-launcher return value: %d\n", *return_values[0]);
		fprintf (stdout, "FlightGear-comunicator return value: %d\n", *return_values[1]);
		fflush(stdout);
	}
	
	return 0;
}
