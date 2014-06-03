// console_controller.c

#include <semaphore.h>
#include <poll.h>

#include "console_controller.h"
#include "../common.h"
#include "../time/drv_time.h"
#include "../display/GUI/GUI.h"
#include "../io_ctrl/comunicator.h"
#include "../../mission/mission.h"


#define MAX_SHELL_COMMAND_LENGTH		25
#ifndef STDIN
	#define STDIN	0
#endif


pthread_t console_thread_id;
sem_t console_semaphore;


typedef enum console_command_name_t
{
	console_command_start_gui,
	console_command_stop_gui,
	console_command_stop_sending_controls,
	console_command_start_sending_controls,
	console_command_set_verbose,
	console_command_unset_verbose,
	console_command_set_very_verbose,
	console_command_unset_very_verbose,	
	console_command_main_quit,
	console_command_help,
	console_command_unknown
} console_command_name_t;


void console_usage ()
{
	printf ("USAGE:\n");
	printf ("   UAV_autopilot <options>\n");
	printf ("ACCEPTED COMMANDS:\n");
	printf ("   start-gui\n");
	printf ("\t Start the GUI if not started (the Xserver must be already running)\n");
	printf ("   stop-gui\n");
	printf ("\t Stop the GUI if started\n");
	printf ("   stop-sending-controls\n");
	printf ("\t Stop sending flight-controls to FlightGear\n");
	printf ("   start-sending-controls\n");
	printf ("\t Start sending flight-controls to FlightGear\n");
	printf ("   set-verbose\n");
	printf ("\t Display more messages\n");
	printf ("   unset-verbose\n");
	printf ("\t Stop displaying verbose messages\n");
	printf ("   set-very-verbose\n");
	printf ("\t Display even more messages\n");
	printf ("   unset-very-verbose\n");
	printf ("\t Stop display vverbose messages\n");
	
	// not allowed in simulator mode
	if (!getenv("START_SIMULATOR"))
	{
		printf ("   quit\n");
		printf ("\t Shutdown the application\n");
	}
	
	printf ("   help\n");
	printf ("\t Show this help\n");
	printf ("\n Press enter to continue\n");
	fflush (stdout);
}

console_command_name_t console_command_decode (char *console_command)
{
	console_command_name_t s_command = console_command_unknown;
	
	if (!strcmp(console_command, "start-gui"))
		s_command = console_command_start_gui;	else if (!strcmp(console_command, "stop-gui"))
		s_command = console_command_stop_gui;
	else if (!strcmp(console_command, "stop-sending-controls"))
		s_command = console_command_stop_sending_controls;
	else if (!strcmp(console_command, "start-sending-controls"))
		s_command = console_command_start_sending_controls;
	else if (!strcmp(console_command, "set-verbose"))
		s_command = console_command_set_verbose;
	else if (!strcmp(console_command, "unset-verbose"))
		s_command = console_command_unset_verbose;
	else if (!strcmp(console_command, "set-very-verbose"))
		s_command = console_command_set_very_verbose;
	else if (!strcmp(console_command, "unset-very-verbose"))
		s_command = console_command_unset_very_verbose;
	else if (!strcmp(console_command, "quit"))
		s_command = console_command_main_quit;
	else if (!strcmp(console_command, "help"))
		s_command = console_command_help;

	return s_command;
}

int get_console_unique_control ()
{
	absolute_time usec_wait_time = 250000;
	int timedwait_return_value;
	struct timespec max_time;
	
	do
	{
		absolute_time_to_timespec (get_absolute_time () + usec_wait_time, &max_time);
		timedwait_return_value = sem_timedwait (&console_semaphore, (const struct timespec *) &max_time);
		if (_shutdown_all_systems)
			return -1;
	}
	while (timedwait_return_value != 0);

	return 0;
}

void release_console_control ()
{
	sem_post (&console_semaphore);
}



void* console_controller_loop (void* args)
{
	console_command_name_t s_command;
	char console_command [MAX_SHELL_COMMAND_LENGTH];
	int poll_return_value, poll_timeout = 1500; // poll_timeout in ms
	struct pollfd fd;
	
	fd.fd = STDIN;
	fd.events = POLLIN;
	fd.revents = 0;

	
	sem_init (&console_semaphore, 0, 1);
	memset (console_command, '\0', MAX_SHELL_COMMAND_LENGTH);

	fprintf (stdout, "Shell controller started, press enter to activate\n");
	if (!getenv("START_SIMULATOR"))
		fprintf (stdout, "Type \'quit\' to exit\n");
	fflush (stdout);
	
	while (!_shutdown_all_systems)
	{
		// wait for activation
		do
		{
			poll_return_value = poll (&fd, 1, poll_timeout);

			if (_shutdown_all_systems)
				return 0;

			if (poll_return_value < 0)
			{
				fprintf (stderr, "Console controller experienced an error while waiting for an input\n");
				return 0;
			}
			if (poll_return_value > 0 && (fd.revents & POLLIN) != 0)
				break;
		}
		while (1);

		fgets (console_command, MAX_SHELL_COMMAND_LENGTH, stdin);


		// stop the printer_thread
		get_console_unique_control ();
		
		fprintf (stdout, "Waiting for a command: ");
		fflush (stdout);

		// wait for command (user input)
		do
		{
			poll_return_value = poll (&fd, 1, poll_timeout);

			if (_shutdown_all_systems)
			{
				//restart the printer_thread
				release_console_control ();
				return 0;
			}

			if (poll_return_value < 0)
			{
				fprintf (stderr, "Console controller experienced an error while waiting for an input\n");
				return 0;
			}
			if (poll_return_value > 0 && (fd.revents & POLLIN) != 0)
				break;
		}
		while (1);

		fgets (console_command, MAX_SHELL_COMMAND_LENGTH, stdin);


		*(console_command + strlen(console_command) -2) = '\0';
		s_command = console_command_decode (console_command);
		
		switch (s_command)
		{
#ifndef BYPASS_OUTPUT_CONTROLS_AND_DO_UAV_MODEL_TEST_DEMO
			case console_command_start_gui:
				// start GUI
				if (getenv("GUI_INIT_ERROR"))
				{
					fprintf (stdout, "The Xserver returned an error so the GUI can't be started until system restart\n");
					break;
				}

				if (getenv("START_GUI"))
				{
					fprintf (stdout, "GUI already started\n");
					break;
				}
				
				setenv ("START_GUI", "", 0);
				if (pthread_create (&GUI_thread_id, NULL, start_GUI, NULL) != 0)
					fprintf (stderr, "Can't create a thread to start the GUI (errno: %d)\n", errno);
				else
					fprintf (stdout, "GUI started\n");
				
				break;
				
			case console_command_stop_gui:
				if (!getenv("START_GUI"))
				{
					fprintf (stdout, "GUI not started\n");
					break;
				}
				
				close_GUI ();
				fprintf (stdout, "GUI closed\n");
				
				break;
#endif
			case console_command_stop_sending_controls:
				if  (!getenv("DO_NOT_SEND_CONTROLS"))
				{
					setenv ("DO_NOT_SEND_CONTROLS", "", 0);
					fprintf (stdout, "No more flight controls will be sent\n");
				}
				else
					fprintf (stdout, "Flight controls are already not sent\n");
				
				break;
			case console_command_start_sending_controls:
				if  (getenv("DO_NOT_SEND_CONTROLS"))
				{
					unsetenv ("DO_NOT_SEND_CONTROLS");
					fprintf (stdout, "Flight controls will now be sent\n");
				}
				else
					fprintf (stdout, "Flight controls are already sent\n");
				
				break;
			case console_command_set_verbose:
				if  (!getenv("VERBOSE"))
				{
					setenv ("VERBOSE", "", 0);
					fprintf (stdout, "Verbose mode activate\n");
				}
				else
					fprintf (stdout, "Verbose mode already activate\n");
				
				break;
			case console_command_unset_verbose:
				if  (getenv("VERBOSE"))
				{
					unsetenv ("VERBOSE");
					fprintf (stdout, "Verbose mode not activate\n");
				}
				else
					fprintf (stdout, "Verbose mode already not activate\n");
				
				break;
			case console_command_set_very_verbose:
				if  (!getenv("VERY_VERBOSE"))
				{
					setenv ("VERBOSE", "", 0);
					setenv ("VERY_VERBOSE", "", 0);
					fprintf (stdout, "Very_verbose mode activate\n");
				}
				else
					fprintf (stdout, "Very_verbose mode already activate\n");
				
				break;
			case console_command_unset_very_verbose:
				if  (getenv("VERY_VERBOSE"))
				{
					unsetenv ("VERY_VERBOSE");
					fprintf (stdout, "Very_verbose mode not activate\n");
				}
				else
					fprintf (stdout, "Very_verbose mode already not activate\n");
				
				break;
			case console_command_main_quit:
				// not allowed in simulator mode
				if (!getenv("START_SIMULATOR"))
				{
					fprintf (stdout, "Exiting\n");
					_shutdown_all_systems = 1;
				}
				else
					fprintf (stdout, "Command unknown\n");
				
				break;
			case console_command_help:
				console_usage ();
				fgets (console_command, MAX_SHELL_COMMAND_LENGTH, stdin);
				
				break;

			case console_command_unknown:
			default: 
				fprintf (stdout, "Command unknown\n");
		}
		
		fprintf (stdout, "\n");
		fflush (stdout);
		
		// let the user read
		if (getenv("VERY_VERBOSE") && s_command != console_command_help)
			sleep (2);
		
		//restart the printer_thread
		release_console_control ();
	}
	
	return 0;
}
