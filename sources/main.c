// main.c

#include <getopt.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <pthread.h>
#include <gtk/gtk.h>
#include <libxml/parser.h>

#include "uav_library/common.h"
#include "uav_library/param/param.h"
#include "uav_library/io_ctrl/socket_io.h"
#include "uav_library/io_ctrl/comunicator.h"
#include "uav_library/time/drv_time.h"
#include "uav_library/display/printer_loop.h"
#include "uav_library/display/GUI/GUI.h"
#include "uav_library/console_controller/console_controller.h"

#include "simulator/comunicator_loop.h"
#include "simulator/FlightGear_launcher.h"
#include "autopilot/autopilot.h"
#include "mission/mission.h"
#include "ORB/ORB.h"
#include "test_thread.h"
#include "uav_type.h"


//#define START_AUTOPILOT_WHEN_SIMULATOR_IS_READY

char *mission_file_name = NULL;
char *aircraft_model_name = NULL;
char *aircraft_model_option = NULL;
bool_t is_rotary_wing = 1 /* true */;
rotor_geometry r_geometry = 0;

int _shutdown_all_systems = 0;



void usage ()
{
	printf ("USAGE:\n");
	printf ("   UAV_autopilot <options>\n");
	printf ("OPTIONS:\n");
	printf ("   -i --input-port\n");
	printf ("\t Set the port where to receive flight-data from FlightGear\n");
	printf ("   -o --output-port\n");
	printf ("\t Set the port where to send the flight-controls for FlightGear\n");
	printf ("   -t --tcp\n");
	printf ("\t Use TCP connection\n");
	printf ("   -u --udp\n");
	printf ("\t Use UDP connection [DEFAULT]\n");
	printf ("   -g --gui\n");
	printf ("\t Start the GUI (the Xserver must be already running - to start it launch startxwin from cmd)\n");
	printf ("   -s --simulator\n");
	printf ("\t Start FlightGear simulator with the correct options\n");
	printf ("   -a --aircraft\n");
	printf ("\t Set the aircraft model to be used in FlightGear]\n");
	printf ("\t Supported models: quadcopter [DEFAULT], rascal\n");
	printf ("   -m --mission\n");
	printf ("\t Set the file containing the mission to be executed\n");
	// printf ("   -d --do-not-send-command\n");
	// printf ("\t Don't send flight-controls to FlightGear\n");
	printf ("   -v --verbose\n");
	printf ("\t Display more messages\n");
	printf ("   -w --very_verbose\n");
	printf ("\t Display even more messages\n");
	printf ("   -h --help\n");
	printf ("\t Show this help\n");
	printf ("\n");
	fflush (stdout);
}

int check_arguments (int argc, char **argv)
{
	int option_index = 0, option_character;
	int input_port_set = 0, output_port_set = 0;
	static struct option long_options[] = {
		{"input-port",				required_argument,	0,	'i'},
		{"output-port",				required_argument,	0,	'o'},
		{"tcp",						no_argument,		0,	't'},
		{"udp",						no_argument,		0,	'u'},
		{"gui",						no_argument,		0,	'g'},
		{"aircraft",				required_argument,	0,	'a'},
		{"simulator",				no_argument,		0,	's'},
		{"mission",					required_argument,	0,	'm'},
		// {"do-not-send-controls",	no_argument,		0,	'd'},
		{"verbose",					no_argument,		0,	'v'},
		{"very_verbose",			no_argument,		0,	'w'},
		{"help",					no_argument,		0,	'h'}
	};
 
	while ((option_character = getopt_long (argc, argv, "i:o:a:tugsm:dvwh", long_options, &option_index)) != -1)
	{
		switch (option_character)
		{
			case 'i': //set input_port
				input_port = atoi (optarg);
				input_port_set = 1;
				break;
				
			case 'o': //set output_port
				output_port = atoi (optarg);
				output_port_set = 1;
				break;
				
			case 'a': //set aircraft_model
				aircraft_model_name = strdup (optarg);
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

			// case 'd': //set do_not_send_controls_flag
				// setenv ("DO_NOT_SEND_CONTROLS", "", 0);
				// break;
			
			case 'v': //set verbose_flag
				setenv ("VERBOSE", "", 0);
				break;

			case 'w': //set verbose_flag
				setenv ("VERBOSE", "", 0);
				setenv ("VERY_VERBOSE", "", 0);
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
	
	if (input_port_set == 0)
		input_port = (getenv("TCP_FLAG"))? DEFAULT_TCP_INPUT_PORT : DEFAULT_UDP_INPUT_PORT;
	if (input_port < 1023 ||  input_port > 65535)
		return -1;
	
	if (output_port_set == 0)
		output_port = (getenv("TCP_FLAG"))? DEFAULT_TCP_OUTPUT_PORT : DEFAULT_UDP_OUTPUT_PORT;
	if (output_port < 1023 ||  output_port > 65535 || output_port == input_port)
		return -1;
	
	strcpy (output_address, DEFAULT_IP_ADDRESS);
	

	if (!aircraft_model_name)
	{
		aircraft_model_name = strdup ((char *) DEFAULT_AIRCRAFT_MODEL);
		r_geometry = QUAD_X;
	}
	if (!strcmp (aircraft_model_name, "quadcopter"))
	{
		aircraft_model_option = strdup (" --aircraft=quadra");
	}
	/*
	else if (!strcmp (aircraft_model_name, "arducopter"))
	{
		aircraft_model_option = strdup (" --aircraft=arducopter");
		geometry = QUAD_X;
	}
	else if (!strcmp (aircraft_model_name, "easystar"))
	{
		is_rotary_wing = 0;
		aircraft_model_option = strdup (" --aircraft=EasyStar");
	}
	*/
	else if (!strcmp (aircraft_model_name, "rascal"))
	{
		is_rotary_wing = 0;
		aircraft_model_option = strdup (" --aircraft=Rascal110-JSBSim");
	}
	else if (!strcmp (aircraft_model_name, "Cessna"))
	{
		is_rotary_wing = 0;
		aircraft_model_option = strdup (" --aircraft=c172p-2dpanel");
	}
	else
	{
		fprintf (stderr, "CRITICAL: Aircraft model %s unsupported\n", aircraft_model_name);
		return -1;
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
	//_shutdown_all_systems = 1;

	// Clean up
	mission_destroy ();

	fflush (stderr);
	fflush (stdout);

	sleep (0.2);
	fprintf (stdout, "Done.\n");
	fflush (stdout);
}


int main (int n_args, char** args)
{
	int *thread_return_values[3];

	if (check_arguments (n_args, args) < 0)
	{
		fprintf (stderr, "Bad args - Can't run the program\n");
		usage();
		exit(-1);
	}

	atexit (do_before_exiting);
	sleep(1);
	
	// ************************************* system init ****************************************
	//if (getenv("START_GUI"))
		gdk_threads_init ();

	/*
	 * this initialize the library and check potential ABI mismatches
	 * between the version it was compiled for and the actual shared
	 * library used.
	 */
	//if (getenv("MISSION_SET"))
		LIBXML_TEST_VERSION
	
	system_time_init ();
	system_orb_init ();
	param_init ();

	fprintf (stdout, "\n");
	fflush (stdout);

	if (mission_init (mission_file_name) != 0)
	{
		fprintf (stderr, "Impossible to initialize the mission\n");
		exit(-1);
	}


	// ************************************* start all system threads ****************************************
	// start console controller
	if (pthread_create (&console_thread_id, NULL, console_controller_loop, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to interact whit the application (errno: %d)\n", errno);
		exit(-1);
	}


	// start primary comunicator loop
	// it must be before start simulator for tcp connections
	if (pthread_create (&comunicator_thread_id, NULL, comunicator_loop, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to comunicate with FlightGear (errno: %d)\n", errno);
		exit(-1);
	}

	// start simulator
	if (getenv("START_SIMULATOR") && pthread_create (&simulator_thread_id, NULL, FlightGear_launcher, aircraft_model_option) != 0)
	{
		fprintf (stderr, "Can't create a thread to launch FlightGear (errno: %d)\n", errno);
		exit(-1);
	}
	else if (!getenv("START_SIMULATOR"))
	{
		// export notification of hil off
		fprintf (stderr, "trying to define a param\n");
		param_define_int("HIL_ENABLED", 0);
		fprintf (stderr, "param defined \n");
	}

	if (getenv("START_SIMULATOR"))
		sleep (5);


	// start GUI
	if (getenv("START_GUI") &&  pthread_create (&GUI_thread_id, NULL, start_GUI, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to start the GUI (errno: %d)\n", errno);
		exit(-1);
	}
	
#ifdef START_AUTOPILOT_WHEN_SIMULATOR_IS_READY
	if (getenv("START_SIMULATOR"))
	{
		while (!simulator_ready)
			sleep (0.3);
	}
#endif

	// start autopilot
	if (pthread_create (&autopilot_thread_id, NULL, autopilot_loop, NULL) != 0)
	{
		fprintf (stderr, "Can't create an autopilot thread to compute the flight controls (errno: %d)\n", errno);
		exit(-1);
	}

	// start printer loop
	if (pthread_create (&printer_thread_id, NULL, printer_loop, NULL) != 0)
	{
		fprintf (stderr, "Can't create a thread to print the aircraft status (errno: %d)\n", errno);
		exit(-1);
	}
	
	sleep(3);

	// start test thread
	if (START_TEST_THREAD && pthread_create (&test_thread_id, NULL, test_thread_body, NULL) != 0)
	{
		fprintf (stderr, "Can't create the test thread (errno: %d)\n", errno);
		exit(-1);
	}


	// ************************************* wait for all threads to exit ****************************************
	// if the simulator mode is active then the main quit will arrive on simulator exit
	// else the main quit must arrive from console input
	if (getenv("START_SIMULATOR"))
	{
		pthread_join(simulator_thread_id, (void**)&(thread_return_values[0]));
		if (*(thread_return_values[0]) != 0)
			fprintf (stderr, "\n\nCRITICAL: FlightGear simulator crushed\n");

		_shutdown_all_systems = 1;
	}

	pthread_join(console_thread_id, NULL);


	if (getenv("START_GUI") && !getenv("GUI_STOPPED"))
	{
		close_GUI ();
		pthread_join(GUI_thread_id, (void**)&(thread_return_values[1]));
	}


	if (input_socket >= 0)
	{
		shutdown (input_socket, SHUT_RDWR);
		close (input_socket);
	}

	if (!getenv("DO_NOT_SEND_CONTROLS") && output_socket >= 0)
	{
		shutdown (output_socket, SHUT_RDWR);
		close (output_socket);
	}

	pthread_join (comunicator_thread_id, (void**)&(thread_return_values[2]));



	// ************************************* print some data before exit ****************************************
	fprintf (stdout, "\n\n");
	if (getenv("VERBOSE"))
	{
		if (getenv("START_SIMULATOR"))
			fprintf (stdout, "FlightGear-launcher return value: %d\n", *(thread_return_values[0]));
		if (getenv("START_GUI") || getenv("GUI_STOPPED"))
			fprintf (stdout, "GUI-launcher return value: %d\n", *(thread_return_values[1]));

		fprintf (stdout, "Comunicator thread return value: %d\n", *(thread_return_values[2]));
	}

	fflush(stdout);

	exit (0);
}
