// plot.c

#include <semaphore.h>
#include <pthread.h>

#include "printer_loop.h"
#include "../common.h"
#include "../time/drv_time.h"
#include "../console_controller/console_controller.h"

#include "../../ORB/ORB.h"
#include "../../ORB/topics/sensors/airspeed.h"
#include "../../ORB/topics/vehicle_hil_attitude.h"
#include "../../ORB/topics/position/vehicle_global_position.h"
#include "../../ORB/topics/actuator/actuator_effective_controls.h"

pthread_t printer_thread_id;


void* printer_loop (void* args)
{
	/* Subscribe to airspeed topic */
	orb_subscr_t airspeed_sub;
	struct airspeed_s airspeed;
	/* Subscribe to vehicle_global_position topic */
	orb_subscr_t vehicle_global_position_sub;
	struct vehicle_global_position_s vehicle_global_position;
	/* Subscribe to vehicle_hil_attitude topic */
	orb_subscr_t vehicle_attitude_sub;
	struct vehicle_hil_attitude_s vehicle_attitude;
	/* Subscribe to actuator_effective_controls topic */
	orb_subscr_t actuator_effective_controls_sub;
	struct actuator_effective_controls_s actuator_effective_controls;
	
	int poll_return_value;
	absolute_time max_update_interval = 2000000;
	

	// ********************** Subscribe to the topics ******************************
	airspeed_sub = orb_subscribe (ORB_ID(airspeed));
	if (airspeed_sub < 0)
	{
		fprintf (stderr, "Printer thread can't subscribe to airspeed topic\n");
		return 0;
	}
	
	vehicle_global_position_sub = orb_subscribe (ORB_ID(vehicle_global_position));
	if (vehicle_global_position_sub < 0)
	{
		fprintf (stderr, "Printer thread can't subscribe to vehicle_global_position topic\n");
		return 0;
	}
	
	vehicle_attitude_sub = orb_subscribe (ORB_ID(vehicle_hil_attitude));
	if (vehicle_attitude_sub < 0)
	{
		fprintf (stderr, "Printer thread can't subscribe to vehicle_hil_attitude topic\n");
		return 0;
	}
	
	actuator_effective_controls_sub = orb_subscribe (ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE);
	if (actuator_effective_controls_sub < 0)
	{
		fprintf (stderr, "Printer thread can't subscribe to actuator_effective_controls topic\n");
		return 0;
	}

	
	// ********************** Set max update intervals ******************************
	if (orb_set_interval (ORB_ID(airspeed), airspeed_sub, max_update_interval) < 0)
	{
		fprintf (stdout, "Failed to set max update interval for airspeed topic\n");
		return 0;
	}
	
	if (orb_set_interval (ORB_ID(vehicle_global_position), vehicle_global_position_sub, max_update_interval) < 0)
	{
		fprintf (stdout, "Failed to set max update interval for vehicle_global_position topic\n");
		return 0;
	}
	
	if (orb_set_interval (ORB_ID(vehicle_hil_attitude), vehicle_attitude_sub, max_update_interval) < 0)
	{
		fprintf (stdout, "Failed to set max update interval for vehicle_hil_attitude topic\n");
		return 0;
	}
	
	if (orb_set_interval (ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE, actuator_effective_controls_sub, max_update_interval) < 0)
	{
		fprintf (stdout, "Failed to set max update interval for actuator_effective_controls topic\n");
		return 0;
	}
	
	// ********************** Start the loop ******************************
	while (!_shutdown_all_systems)
	{
		if (!getenv("VERY_VERBOSE"))
		{
			sleep (5);
			continue;		
		}
		
		poll_return_value = orb_poll (ORB_ID(airspeed), airspeed_sub, 1000000);
		if (poll_return_value < 0)
		{
			fprintf (stderr, "Printer thread experienced an error waiting for airspeed topic\n");
			continue;
		}
		else if (poll_return_value)
			orb_copy (ORB_ID(airspeed), airspeed_sub, (void *) &airspeed);
		
		poll_return_value = orb_poll (ORB_ID(vehicle_global_position), vehicle_global_position_sub, 1000000);
		if (poll_return_value < 0)
		{
			fprintf (stderr, "Printer thread experienced an error waiting for vehicle_global_position topic\n");
			continue;
		}
		else if (poll_return_value)
			orb_copy (ORB_ID(vehicle_global_position), vehicle_global_position_sub, (void *) &vehicle_global_position);
		
		poll_return_value = orb_poll (ORB_ID(vehicle_hil_attitude), vehicle_attitude_sub, 1000000);
		if (poll_return_value < 0)
		{
			fprintf (stderr, "Printer thread experienced an error waiting for vehicle_hil_attitude topic\n");
			continue;
		}
		else if (poll_return_value)
			orb_copy (ORB_ID(vehicle_hil_attitude), vehicle_attitude_sub, (void *) &vehicle_attitude);
		
		if (!getenv("DO_NOT_SEND_CONTROLS"))
		{
			poll_return_value = orb_poll (ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE, actuator_effective_controls_sub, 1000000);
			if (poll_return_value < 0)
			{
				fprintf (stderr, "Printer thread experienced an error waiting for actuator_effective_controls topic\n");
				continue;
			}
			else if (poll_return_value)
				orb_copy (ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE, actuator_effective_controls_sub, (void *) &actuator_effective_controls);
			
		}
		
		// do not plot until the user has finish to control the console
		get_console_unique_control ();
		
		//Print data to screen for monitoring purposes
		// INPUTS
		fprintf (stdout, "\nFlight-data\n\
				Latitude: %.10f,\tLongitude: %.10f,\tAltitude: %.6f,\tGround level: %.6f\n\
				Roll angle: %.6f,\tPitch angle: %.6f,\tYaw angle: %.6f\n\
				Roll rate: %.6f,\tPitch rate: %.6f,\tYaw rate: %.6f\n\
				X body velocity: %.6f,\tV body velocity: %.6f,\tW body velocity: %.6f\n\
				X earth velocity: %.6f,\tY earth velocity: %.6f,\tZ earth velocity: %.6f,\tAirspeed: %.6f\n\
				X body acceleration: %.6f,\tY body acceleration: %.6f,\tZ body acceleration: %.6f\n\
				Engine rotation speed: %.6f,\tEngine thrust: %.6f\n",
				((float)vehicle_global_position.latitude)  / 1e7f, ((float) vehicle_global_position.longitude) / 1e7f,	vehicle_global_position.altitude, vehicle_global_position.ground_level,
				vehicle_attitude.roll, vehicle_attitude.pitch, vehicle_attitude.yaw,
				vehicle_attitude.roll_rate, vehicle_attitude.pitch_rate, vehicle_attitude.yaw_rate,
				vehicle_attitude.vx, vehicle_attitude.vy, vehicle_attitude.vz,
				vehicle_global_position.vx, vehicle_global_position.vy, vehicle_global_position.vz, airspeed.indicated_airspeed_m_s,
				vehicle_attitude.ax, vehicle_attitude.ay, vehicle_attitude.az,
				vehicle_attitude.engine_rotation_speed, vehicle_attitude.thrust);
		fflush (stdout);
		
		// OUTPUTS
		fprintf (stdout, "\nFlight_controls\n\
				Aileron:\t%.6f\n\
				Elevator:\t%.6f\n\
				Rudder:\t\t%.6f\n\
				Throttle:\t%.6f\n",
				actuator_effective_controls.control[0],
				actuator_effective_controls.control[1],
				actuator_effective_controls.control[2],
				actuator_effective_controls.control[3]);
		fflush (stdout);

		// allow the user to control the console
		release_console_control ();
	}
	

	// ************************************* unsubscribe ******************************************
	if (orb_unsubscribe (ORB_ID(airspeed), airspeed_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to airspeed topic\n");

	if (orb_unsubscribe (ORB_ID(vehicle_global_position), vehicle_global_position_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to vehicle_global_position topic\n");

	if (orb_unsubscribe (ORB_ID(vehicle_hil_attitude), vehicle_attitude_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to vehicle_hil_attitude topic\n");

	if (orb_unsubscribe (ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE, actuator_effective_controls_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to actuator_effective_controls topic\n");
	
	return 0;
}
