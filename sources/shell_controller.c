// shell_controller.c

#include <semaphore.h>
#include "shell_controller.h"
#include "common.h"
#include "comunicator.h"
#include "autopilot.h"
#include "aircraft.h"
#include "GUI.h"
#include "drv_time.h"

#define MAX_SHELL_COMMAND_LENGTH		25

pthread_t shell_thread_id;
sem_t shell_semaphore;


typedef enum shell_command_name_t
{
	shell_command_start_gui,
	shell_command_stop_gui,
	shell_command_restrat_mission,
	shell_command_stop_sending_controls,
	shell_command_start_sending_controls,
	shell_command_set_verbose,
	shell_command_unset_verbose,
	shell_command_set_very_verbose,
	shell_command_unset_very_verbose,	
	shell_command_main_quit,
	shell_command_help,
	shell_command_unknown
} shell_command_name_t;


void shell_usage ()
{
	printf ("USAGE:\n");
	printf ("   UAV_autopilot <options>\n");
	printf ("ACCEPTED COMMANDS:\n");
	printf ("   start-gui\n");
	printf ("\t Start the GUI if not started (the Xserver must be already running)\n");
	printf ("   stop-gui\n");
	printf ("\t Stop the GUI if started\n");
	printf ("   restart-mission\n");
	printf ("\t Restart the mission if set\n");
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

shell_command_name_t shell_command_decode (char *shell_command)
{
	shell_command_name_t s_command = shell_command_unknown;
	
	if (!strcmp(shell_command, "start-gui"))
		s_command = shell_command_start_gui;	else if (!strcmp(shell_command, "stop-gui"))
		s_command = shell_command_stop_gui;
	else if (!strcmp(shell_command, "restart-mission"))
		s_command = shell_command_restrat_mission;
	else if (!strcmp(shell_command, "stop-sending-controls"))
		s_command = shell_command_stop_sending_controls;
	else if (!strcmp(shell_command, "start-sending-controls"))
		s_command = shell_command_start_sending_controls;
	else if (!strcmp(shell_command, "set-verbose"))
		s_command = shell_command_set_verbose;
	else if (!strcmp(shell_command, "unset-verbose"))
		s_command = shell_command_unset_verbose;
	else if (!strcmp(shell_command, "set-very-verbose"))
		s_command = shell_command_set_very_verbose;
	else if (!strcmp(shell_command, "unset-very-verbose"))
		s_command = shell_command_unset_very_verbose;
	else if (!strcmp(shell_command, "quit"))
		s_command = shell_command_main_quit;
	else if (!strcmp(shell_command, "help"))
		s_command = shell_command_help;

	return s_command;
}

void* shell_controller_loop (void* args)
{
	shell_command_name_t s_command;
	char shell_command [MAX_SHELL_COMMAND_LENGTH];
	
	absolute_time usec_wait_time = 250000;
	int timedwait_return_value;
	struct timespec max_time;
	
	sem_init (&shell_semaphore, 0, 1);
	memset (shell_command, '\0', MAX_SHELL_COMMAND_LENGTH);

	fprintf (stdout, "Shell controller started, press enter to activate\n");
	
	while (!_shutdown_all_systems)
	{
		// wait for activation
		fgets (shell_command, MAX_SHELL_COMMAND_LENGTH, stdin);
				
		// stop the printer_thread
		do
		{
			absolute_time_to_timespec (get_absolute_time () + usec_wait_time, &max_time);
			timedwait_return_value = sem_timedwait (&shell_semaphore, (const struct timespec *) &max_time);
			if (_shutdown_all_systems)
				return 0;
		}
		while (timedwait_return_value != 0);
		
		fprintf (stdout, "\nWaiting for a command: ");
		fflush (stdout);
		fgets (shell_command, MAX_SHELL_COMMAND_LENGTH, stdin);
		*(shell_command + strlen(shell_command) -2) = '\0';
		s_command = shell_command_decode (shell_command);
		
		switch (s_command)
		{
			case shell_command_start_gui:
				// start GUI
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
				
			case shell_command_stop_gui:
				if (!getenv("START_GUI"))
				{
					fprintf (stdout, "GUI not started\n");
					break;
				}
				
				close_GUI ();
				fprintf (stdout, "GUI closed\n");
				
				break;
				
			case shell_command_restrat_mission:
				if (mission_restart () < 0)
					fprintf (stderr, "Can't restart the mission\n");
				else
					mission_textview_update ();
				
				break;
				
			case shell_command_stop_sending_controls:
				if  (!getenv("DO_NOT_SEND_CONTROLS"))
				{
					setenv ("DO_NOT_SEND_CONTROLS", "", 0);
					fprintf (stdout, "No more flight controls will be sent\n");
				}
				else
					fprintf (stdout, "Flight controls are already not sent\n");
				
				break;
			case shell_command_start_sending_controls:
				if  (getenv("DO_NOT_SEND_CONTROLS"))
				{
					unsetenv ("DO_NOT_SEND_CONTROLS");
					fprintf (stdout, "Flight controls will now be sent\n");
				}
				else
					fprintf (stdout, "Flight controls are already sent\n");
				
				break;
			case shell_command_set_verbose:
				if  (!getenv("VERBOSE"))
				{
					setenv ("VERBOSE", "", 0);
					fprintf (stdout, "Verbose mode activate\n");
				}
				else
					fprintf (stdout, "Verbose mode already activate\n");
				
				break;
			case shell_command_unset_verbose:
				if  (getenv("VERBOSE"))
				{
					unsetenv ("VERBOSE");
					fprintf (stdout, "Verbose mode not activate\n");
				}
				else
					fprintf (stdout, "Verbose mode already not activate\n");
				
				break;
			case shell_command_set_very_verbose:
				if  (!getenv("VERY_VERBOSE"))
				{
					setenv ("VERBOSE", "", 0);
					setenv ("VERY_VERBOSE", "", 0);
					fprintf (stdout, "Very_verbose mode activate\n");
				}
				else
					fprintf (stdout, "Very_verbose mode already activate\n");
				
				break;
			case shell_command_unset_very_verbose:
				if  (getenv("VERY_VERBOSE"))
				{
					unsetenv ("VERY_VERBOSE");
					fprintf (stdout, "Very_verbose mode not activate\n");
				}
				else
					fprintf (stdout, "Very_verbose mode already not activate\n");
				
				break;
			case shell_command_main_quit:
				// not allowed in simulator mode
				if (!getenv("START_SIMULATOR"))
				{
					if (getenv("START_GUI"))
						close_GUI ();
					
					_shutdown_all_systems = 1;
					fprintf (stdout, "Exiting\n");
				}
				else
					fprintf (stderr, "Command unknown\n");
				
				break;
			case shell_command_help:
				shell_usage ();
				fgets (shell_command, MAX_SHELL_COMMAND_LENGTH, stdin);
				
				break;

			case shell_command_unknown:
			default: 
				fprintf (stdout, "Command unknown\n");
		}
		
		//fprintf (stdout, "\n");
		fflush (stdout);
		
		// let the user read
		if (getenv("VERY_VERBOSE") && s_command != shell_command_help)
			sleep (2);
		
		//restart the printer_thread
		sem_post (&shell_semaphore);
	}
	
	return 0;
}
