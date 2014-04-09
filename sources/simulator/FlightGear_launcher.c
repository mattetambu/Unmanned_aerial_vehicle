// FlightGear_launcher.c

#include <pthread.h>

#include "FlightGear_launcher.h"

#include "../uav_type.h"
#include "../uav_library/common.h"
#include "../uav_library/param/param.h"
#include "../uav_library/io_ctrl/comunicator.h"


pthread_t simulator_thread_id;
int launcher_thread_return_value = 0;

void* FlightGear_launcher (void* args)
{
	int hil_enabled;

	char *aircraft_model_option = strdup ((char *) args);
	char protocol_name[50];

	int command_length = 0;
	char* command;
	char c_input_port[5], c_output_port[5];


	snprintf(c_input_port, 5, "%d", input_port);
	snprintf(c_output_port, 5, "%d", output_port);
	snprintf(protocol_name, 50, "%s_in_out_protocol-rad", (is_rotary_wing)? "multirotor" : "fixedwing");
	
	
#if defined(__CYGWIN__) || defined(CYGWIN) //CYGWIN
	command_length += strlen ((char*) CYGWIN_FLIGHTGEAR_LANCH_COMMAND);
#else //defined(__LINUX__) || defined(LINUX) || defined(linux) || defined(G_OS_UNIX) //UNIX
	command_length += strlen ((char*) UNIX_FLIGHTGEAR_LANCH_COMMAND);
#endif


	command_length += strlen ((char*) FLIGHTGEAR_LANCH_GENERAL_OPTIONS);
	command_length += strlen ((char*) aircraft_model_option);
	command_length += strlen ((char*) FLIGHTGEAR_LANCH_AIRPORT_OPTIONS);
	#ifdef START_IN_AIR
		command_length += strlen ((char*) FLIGHTGEAR_LANCH_IN_AIR_OPTIONS);
	#endif
	command_length += strlen ((char*) " --generic=socket,out,")
					+ strlen ((char*) INPUT_FREQUENCY)
					+ strlen ((char*) ",127.0.0.1,")
					+ strlen ((char*) c_input_port)
					+ strlen ((char*) ",***,") // *** means tcp or udp
					+ strlen ((char*) protocol_name);


	if (!getenv("DO_NOT_SEND_CONTROLS"))
	{
		command_length += strlen ((char*) " --generic=socket,in,")
						+ strlen ((char*) OUTPUT_FREQUENCY)
						+ strlen ((char*) ",127.0.0.1,")
						+ strlen ((char*) c_output_port)
						+ strlen ((char*) ",***,") // *** means tcp or udp
						+ strlen ((char*) protocol_name);
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
#else //defined(__LINUX__) || defined(LINUX) || defined(linux) || defined(G_OS_UNIX) //UNIX
	strcpy (command, (char*) UNIX_FLIGHTGEAR_LANCH_COMMAND);
#endif

	strcat (command, (char*) FLIGHTGEAR_LANCH_GENERAL_OPTIONS);
	strcat (command, (char*) aircraft_model_option);
	strcat (command, (char*) FLIGHTGEAR_LANCH_AIRPORT_OPTIONS);
	#ifdef START_IN_AIR
		strcat (command, (char*) FLIGHTGEAR_LANCH_IN_AIR_OPTIONS);
	#endif
	strcat (command, (char*) " --generic=socket,out,");
	strcat (command, (char*) INPUT_FREQUENCY);
	strcat (command, (char*) ",127.0.0.1,");
	strcat (command, (char*) c_input_port);
	strcat (command, (char*) ",");
	if (getenv("TCP_FLAG"))
		strcat (command, (char*) "tcp,");
	else
		strcat (command, (char*) "udp,");
	strcat (command, (char*) protocol_name);

	if (!getenv("DO_NOT_SEND_CONTROLS"))
	{
		strcat (command, (char*) " --generic=socket,in,");
		strcat (command, (char*) OUTPUT_FREQUENCY);
		strcat (command, (char*) ",127.0.0.1,");
		strcat (command, (char*) c_output_port);
		strcat (command, (char*) ",");
		if (getenv("TCP_FLAG"))
			strcat (command, (char*) "tcp,");
		else
			strcat (command, (char*) "udp,");
		strcat (command, (char*) protocol_name);
	}

	fprintf (stdout, "\nTrying to launch FlightGear ");
	if (getenv("VERBOSE"))
		fprintf (stdout, "using the command:\n%s\n", command);
	fprintf (stdout, "\n");
	fflush(stdout);
	
	// do not remove
	sleep(1);

	// export notification of hil on
	hil_enabled = 1;
	param_define_int("HIL_ENABLED", hil_enabled);

	// start flightgear
	launcher_thread_return_value = system (command);
	free (command);
	
	// export notification of hil off
	/*
	 * param_t hil_param;
	 * hil_enabled = 0;
	 * hil_param = param_find("HIL_ENABLED");
	 * param_set(hil_param, &hil_enabled);
	 */

	pthread_exit(&launcher_thread_return_value);
	return 0;
}
