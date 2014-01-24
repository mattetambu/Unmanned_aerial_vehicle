// plot.c

#include <semaphore.h>
#include "printer_loop.h"
#include "primary_loop.h"
#include "shell_controller.h"
#include "drv_time.h"

#define PRINT_OUT_FREQUENCY		3

pthread_t printer_thread_id;

void plot_flight_data (double* flight_data)
{
	//Print data to screen for monitoring purposes
	fprintf (stdout, "\nFlight-data\n\
			Flight time: %.6f,\tTemperature: %.6f,\tPressure: %.6f\n\
			Latitude: %.10f,\tLongitude: %.10f\n\
			Altitude: %.6f,\tGround level: %.6f\n\
			Roll angle: %.6f,\tPitch angle: %.6f,\tYaw angle: %.6f\n\
			Roll rate: %.6f,\tPitch rate: %.6f,\tYaw rate: %.6f\n\
			U velocity: %.6f,\tV velocity: %.6f,\tW velocity: %.6f\n\
			North velocity: %.6f,\tEast velocity: %.6f,\tDown velocity: %.6f,\tAirspeed: %.6f\n\
			X acceleration: %.6f,\tY acceleration: %.6f,\tZ acceleration: %.6f\n\
			North acceleration: %.6f,\tEast acceleration: %.6f,\tDown acceleration: %.6f\n\
			Engine rotation speed: %.6f\n\
			Magnetic variation: %.6f,\tMagnetic dip: %.6f\n",
			flight_data[FDM_FLIGHT_TIME], flight_data[FDM_TEMPERATURE], flight_data[FDM_PRESSURE],
			flight_data[FDM_LATITUDE], flight_data[FDM_LONGITUDE],
			flight_data[FDM_ALTITIUDE], flight_data[FDM_GROUND_LEVEL],
			flight_data[FDM_ROLL_ANGLE], flight_data[FDM_PITCH_ANGLE], flight_data[FDM_YAW_ANGLE],
			flight_data[FDM_ROLL_RATE], flight_data[FDM_PITCH_RATE], flight_data[FDM_YAW_RATE],
			flight_data[FDM_U_VELOCITY], flight_data[FDM_V_VELOCITY], flight_data[FDM_W_VELOCITY],
			flight_data[FDM_NORTH_VELOCITY], flight_data[FDM_EAST_VELOCITY], flight_data[FDM_DOWN_VELOCITY], flight_data[FDM_AIRSPEED],
			flight_data[FDM_X_ACCELERATION], flight_data[FDM_Y_ACCELERATION], flight_data[FDM_Z_ACCELERATION],
			flight_data[FDM_NORTH_ACCELERATION], flight_data[FDM_EAST_ACCELERATION], flight_data[FDM_DOWN_ACCELERATION],
			flight_data[FDM_ENGINE_RPM],
			flight_data[FDM_MAGNETIC_VARIATION], flight_data[FDM_MAGNETIC_DIP]);

	fflush (stdout);
}

void plot_flight_controls (double* flight_controls)
{
	//Print data to screen for monitoring purposes
	fprintf (stdout, "\nFlight_controls\n\
			Aileron:\t%.6f\n\
			Elevator:\t%.6f\n\
			Rudder:\t\t%.6f\n\
			Throttle:\t%.6f\n",
			flight_controls[CTRL_AILERON],
			flight_controls[CTRL_ELEVATOR],
			flight_controls[CTRL_RUDDER],
			flight_controls[CTRL_THROTTLE]);

	fflush (stdout);
}


void* printer_loop (void* args)
{
	absolute_time usec_wait_time = 250000;
	int timedwait_return_value;
	struct timespec max_time;
	
	
	while (!_shutdown_all_systems)
	{
		usleep (1000000/PRINT_OUT_FREQUENCY);
		
		// do not plot until the user has finish to control the shell
		do
		{
			absolute_time_to_timespec (get_absolute_time () + usec_wait_time, &max_time);
			timedwait_return_value = sem_timedwait (&shell_semaphore, (const struct timespec *) &max_time);
			if (_shutdown_all_systems)
				return 0;
		}
		while (timedwait_return_value != 0);
		
		// INPUTS
		if (getenv("VERY_VERBOSE"))
			plot_flight_data (aircraft->flight_data);

		// OUTPUTS
		if (getenv("VERY_VERBOSE") && !getenv("DO_NOT_SEND_CONTROLS"))
			plot_flight_controls (aircraft->flight_controls);
			
		// allow the user to control the shell
		sem_post (&shell_semaphore);
	}
	
	return 0;
}
