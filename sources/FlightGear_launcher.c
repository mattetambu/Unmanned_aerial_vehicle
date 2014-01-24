// FlightGear_launcher.c

#include "FlightGear_launcher.h"

pthread_t simulator_thread_id;
int launcher_thread_return_value = 0;

void* FlightGear_launcher (void* args)
{
	int command_length = 0;
	char* command;
	char c_input_port[5], c_output_port[5];
	
	snprintf(c_input_port, 5, "%d", input_port);
	snprintf(c_output_port, 5, "%d", output_port);
	
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
					+ strlen ((char*) ",***,") // *** means tcp or udp
					+ strlen ((char*) OUTPUT_PROTOCOL_FILE_NAME);

	if (!getenv("DO_NOT_SEND_CONTROLS"))
	{
		command_length += strlen ((char*) " --generic=socket,in,")
						+ strlen ((char*) OUTPUT_FREQUENCY)
						+ strlen ((char*) ",127.0.0.1,")
						+ strlen ((char*) c_output_port)
						+ strlen ((char*) ",***,") // *** means tcp or udp
						+ strlen ((char*) INPUT_PROTOCOL_FILE_NAME);
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
		strcat (command, (char*) "tcp,");
	else
		strcat (command, (char*) "udp,");
	strcat (command, (char*) OUTPUT_PROTOCOL_FILE_NAME);

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
		strcat (command, (char*) INPUT_PROTOCOL_FILE_NAME);
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
