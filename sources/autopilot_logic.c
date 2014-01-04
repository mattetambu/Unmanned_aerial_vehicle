// autopilot_logic.c

#include "common.h"
#include "autopilot_parameters.h"
#include "autopilot_logic.h"

aircraft_t *aircraft;
float DELTA = 0.001;

int aircraft_structure_init (/*aircraft_t *aircraft*/)
{
	aircraft = malloc (sizeof(aircraft_t));
	memset (aircraft, 0, sizeof (aircraft_t));
	
	aircraft->home_WP = malloc (sizeof(map_point_t));
	memset (aircraft->home_WP, 0, sizeof (map_point_t));	
	aircraft->takeoff_WP = malloc (sizeof(map_point_t));
	memset (aircraft->takeoff_WP, 0, sizeof (map_point_t));
	aircraft->WP_to_reach = malloc (sizeof(map_point_t));
	memset (aircraft->WP_to_reach, 0, sizeof (map_point_t));
	
	aircraft->flight_parameters_error = malloc (sizeof(flight_parameters_error_t));
	memset (aircraft->flight_parameters_error, 0, sizeof (flight_parameters_error_t));
	
	if (!aircraft ||
		!aircraft->home_WP ||
		!aircraft->takeoff_WP ||
		!aircraft->WP_to_reach ||
		!aircraft->flight_parameters_error)
	{
		fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
		return -1;
	}
	
	return 0;
}

void aircraft_structure_destroy ()
{
	if (!aircraft)
		return;
		
	free (aircraft->home_WP);
	free (aircraft->takeoff_WP);
	free (aircraft->WP_to_reach);
	free (aircraft->flight_parameters_error);
	free (aircraft);
}

int compute_flight_controls (double* flight_controls, double* flight_data)
{
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
