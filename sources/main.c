// main.c

#include <getopt.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include "common.h"
#include "FlightGear_launcher.h"
#include "socket_io.h"
#include "autopilot_logic.h"
#include "autopilot_parameters.h"
#include "mission_logic.h"
#include "mission_parameters.h"
#include "FlightGear_comunicator.h"
#include "GUI.h"
#include "main.h"

char *mission_file_name = NULL;


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
	printf ("\t Start the GUI (the Xserver must be already running - to start it launch startxwin from cmd)\n");
	printf ("   -s --simulator\n");
	printf ("\t Start FlightGear simulator whit the correct options\n");
	printf ("   -m --mission\n");
	printf ("\t Set the file containing the mission to be executed\n");
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
		{"simulator",				no_argument,		0,	's'},
		{"mission",					required_argument,	0,	'm'},
		{"do-not-send-controls",	no_argument,		0,	'd'},
		{"verbose",					no_argument,		0,	'v'},
		{"help",					no_argument,		0,	'h'}
	};
 
	while ((option_character = getopt_long (argc, argv, "i:o:a:tugsm:dvh", long_options, &option_index)) != -1)
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
				strncpy (output_address, (char*) optarg, 20);
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
				setenv ("START_SIMULATOR", "", 0);
				break;

			case 'm': //set start_flightgear_flag
				setenv ("MISSION_SET", "", 0);
				mission_file_name = strdup (optarg);
				break;

			case 'd': //set do_not_send_controls_flag
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
	
	if (getenv("MISSION_SET"))
	{
		/*
		 * this initialize the library and check potential ABI mismatches
		 * between the version it was compiled for and the actual shared
		 * library used.
		 */
		LIBXML_TEST_VERSION
	}
	
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

void do_before_exiting ()
{
	aircraft_structure_destroy ();
	mission_structure_destroy ();
}


int main (int n_args, char** args)
{
	int* return_values[3];
	pthread_t launcher_thread_ID, comunicator_thread_ID, GUI_thread_ID;
	pthread_mutex_init (&mutex, NULL);
	
	atexit (do_before_exiting);
	if (check_arguments (n_args, args) < 0)
	{
		fprintf (stderr, "Bad args - Can't run the program\n");
		usage();
		exit(-1);
	}

	sleep(3);

	
	if (aircraft_structure_init(/*aircraft*/) < 0)
	{
		fprintf (stderr, "Impossible to initialize the aircraft structure\n");
		exit(-1);
	}
	
	if (mission_structure_init (/*mission, */mission_file_name) != 0)
	{
		fprintf (stderr, "Impossible to initialize the mission structure\n");
		exit(-1);
	}

	if (getenv("START_SIMULATOR") && pthread_create (&launcher_thread_ID, NULL, FlightGear_launcher, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to launch FlightGear (errno: %d)\n", errno);
		exit(-1);
	}

#ifndef DO_NOT_COMUNICATE
	if (pthread_create (&comunicator_thread_ID, NULL, FlightGear_comunicator, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to comunicate whit FlightGear (errno: %d)\n", errno);
		exit(-1);
	}
#endif
	
	if (getenv("START_GUI") &&  pthread_create (&GUI_thread_ID, NULL, start_GUI, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to start the GUI (errno: %d)\n", errno);
		exit(-1);
	}

	if (getenv("START_SIMULATOR"))
		pthread_join(launcher_thread_ID, (void**)&(return_values[0]));
	else if (getenv("START_GUI"))
		pthread_join(GUI_thread_ID, (void**)&(return_values[1]));

#ifndef DO_NOT_COMUNICATE	
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
	pthread_join(comunicator_thread_ID, (void**)&(return_values[2]));
	pthread_mutex_destroy (&mutex);
#endif	
	
	if (getenv("VERBOSE"))
	{
		if (getenv("START_SIMULATOR"))
			fprintf (stdout, "\nFlightGear-launcher return value: %d\n", *return_values[0]);
		if (getenv("START_GUI") || getenv("GUI_STOPPED"))
			fprintf (stdout, "GUI-launcher return value: %d\n", *return_values[1]);

#ifndef DO_NOT_COMUNICATE
		fprintf (stdout, "FlightGear-comunicator return value: %d\n", *return_values[2]);
#endif
		fflush(stdout);
	}
	
	return 0;
}
