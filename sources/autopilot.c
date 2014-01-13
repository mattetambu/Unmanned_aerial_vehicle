// autopilot.c

#include "autopilot.h"


autopilot_t *autopilot = NULL;

int autopilot_init (char *mission_file_name)
{
	autopilot = malloc (sizeof(autopilot_t));
	if (!autopilot)
	{
		fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
		return -1;
	}
	memset (autopilot, 0, sizeof (autopilot_t));
	
	autopilot->home_WP = malloc (sizeof(map_point_t));
	autopilot->takeoff_WP = malloc (sizeof(map_point_t));
	autopilot->WP_to_reach = malloc (sizeof(map_point_t));
	autopilot->flight_parameters_error = malloc (sizeof(flight_parameters_error_t));
	
	if (!autopilot->home_WP ||
		!autopilot->takeoff_WP ||
		!autopilot->WP_to_reach ||
		!autopilot->flight_parameters_error)
	{
		fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
		return -1;
	}
	
	memset (autopilot->home_WP, 0, sizeof (map_point_t));	
	memset (autopilot->takeoff_WP, 0, sizeof (map_point_t));
	memset (autopilot->WP_to_reach, 0, sizeof (map_point_t));
	memset (autopilot->flight_parameters_error, 0, sizeof (flight_parameters_error_t));
	
	if (aircraft_init() < 0)
	{
		fprintf (stderr, "Impossible to initialize the aircraft\n");
		return -1;
	}
	
	if (mission_init (mission_file_name) != 0)
	{
		fprintf (stderr, "Impossible to initialize the mission\n");
		return -1;
	}
	
	return 0;
}

void autopilot_destroy ()
{
	if (!autopilot)
		return;
		
	if (autopilot->home_WP)
		free (autopilot->home_WP);
		
	if (autopilot->takeoff_WP)
		free (autopilot->takeoff_WP);
		
	if (autopilot->WP_to_reach)
		free (autopilot->WP_to_reach);
		
	free (autopilot->flight_parameters_error);
	
	aircraft_destroy ();
	mission_destroy ();
		
	free (autopilot);
}


int compute_flight_controls (double* flight_controls, double* flight_data)
{
	float DELTA = 0.001;
	
	//Proportional control for roll and pitch
	flight_controls[AILERON] = -0.05*(flight_data[ROLL_ANGLE] - 0);
	flight_controls[ELEVATOR] = 0.1*(flight_data[PITCH_ANGLE] - 5);
	flight_controls[RUDDER] = 0;

	//Set the engine throttle as a sinusoidal function
	flight_controls[THROTTLE] += DELTA;

	//Limit control inputs
	if (fabs(flight_controls[AILERON]) > 0.6)
			flight_controls[AILERON] = (flight_controls[AILERON]/fabs(flight_controls[AILERON]))*0.6;
	if (fabs(flight_controls[ELEVATOR]) > 0.6)
			flight_controls[ELEVATOR] = (flight_controls[ELEVATOR]/fabs(flight_controls[ELEVATOR]))*0.6;
	if ((flight_controls[THROTTLE] >= 0.95 && DELTA > 0) || (flight_controls[THROTTLE] <= 0.5 && DELTA < 0))
		DELTA = -DELTA;

	return 0;
}
