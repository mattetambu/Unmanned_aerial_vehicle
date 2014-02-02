// plot.c

#include <semaphore.h>
#include <pthread.h>

#include "printer_loop.h"
#include "../common.h"
#include "../time/drv_time.h"
#include "../console_controller/console_controller.h"

#include "../../ORB/ORB.h"
#include "../../ORB/topics/airspeed.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/vehicle_global_position.h"
#include "../../ORB/topics/actuator/actuator_controls.h"

pthread_t printer_thread_id;


void* printer_loop (void* args)
{
	/* Subscribe to airspeed topic */
	orb_subscr_t airspeed_sub;
	struct airspeed_s local_airspeed;
	/* Subscribe to vehicle_global_position topic */
	orb_subscr_t vehicle_global_position_sub;
	struct vehicle_global_position_s local_vehicle_global_position;
	/* Subscribe to vehicle_attitude topic */
	orb_subscr_t vehicle_attitude_sub;
	struct vehicle_attitude_s local_vehicle_attitude;
	/* Subscribe to actuator_controls topic */
	orb_subscr_t actuator_controls_sub;
	struct actuator_controls_s local_actuator_controls;
	
	int wait_return_value;
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
	
	vehicle_attitude_sub = orb_subscribe (ORB_ID(vehicle_attitude));
	if (vehicle_attitude_sub < 0)
	{
		fprintf (stderr, "Printer thread can't subscribe to vehicle_attitude topic\n");
		return 0;
	}
	
	actuator_controls_sub = orb_subscribe (ORB_ID(actuator_controls));
	if (actuator_controls_sub < 0)
	{
		fprintf (stderr, "Printer thread can't subscribe to vehicle_attitude topic\n");
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
	
	if (orb_set_interval (ORB_ID(vehicle_attitude), vehicle_attitude_sub, max_update_interval) < 0)
	{
		fprintf (stdout, "Failed to set max update interval for vehicle_attitude topic\n");
		return 0;
	}
	
	if (orb_set_interval (ORB_ID(actuator_controls), actuator_controls_sub, max_update_interval) < 0)
	{
		fprintf (stdout, "Failed to set max update interval for vehicle_attitude topic\n");
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
		
		wait_return_value = orb_wait (ORB_ID(airspeed), airspeed_sub);
		if (wait_return_value < 0)
		{
			fprintf (stderr, "Printer thread experienced an error waiting for airspeed topic\n");
			break;
		}
		else if (wait_return_value)
			orb_copy (ORB_ID(airspeed), airspeed_sub, (void *) &local_airspeed);
		
		wait_return_value = orb_wait (ORB_ID(vehicle_global_position), vehicle_global_position_sub);
		if (wait_return_value < 0)
		{
			fprintf (stderr, "Printer thread experienced an error waiting for vehicle_global_position topic\n");
			break;
		}
		else if (wait_return_value)
			orb_copy (ORB_ID(vehicle_global_position), vehicle_global_position_sub, (void *) &local_vehicle_global_position);
		
		wait_return_value = orb_wait (ORB_ID(vehicle_attitude), vehicle_attitude_sub);
		if (wait_return_value < 0)
		{
			fprintf (stderr, "Printer thread experienced an error waiting for vehicle_attitude topic\n");
			break;
		}
		else if (wait_return_value)
			orb_copy (ORB_ID(vehicle_attitude), vehicle_attitude_sub, (void *) &local_vehicle_attitude);
		
		if (!getenv("DO_NOT_SEND_CONTROLS"))
		{
			wait_return_value = orb_wait (ORB_ID(actuator_controls), actuator_controls_sub);
			if (wait_return_value < 0)
			{
				fprintf (stderr, "Printer thread experienced an error waiting for actuator_controls topic\n");
				break;
			}
			else if (wait_return_value)
				orb_copy (ORB_ID(actuator_controls), actuator_controls_sub, (void *) &local_actuator_controls);
			
		}
		
		// do not plot until the user has finish to control the console
		get_console_unique_control ();
		
		//Print data to screen for monitoring purposes
		// INPUTS
		fprintf (stdout, "\nFlight-data\n\
				Latitude: %.10f,\tLongitude: %.10f\n\
				Altitude: %.6f,\tGround level: %.6f\n\
				Roll angle: %.6f,\tPitch angle: %.6f,\tYaw angle: %.6f\n\
				Roll rate: %.6f,\tPitch rate: %.6f,\tYaw rate: %.6f\n\
				X body velocity: %.6f,\tV body velocity: %.6f,\tW body velocity: %.6f\n\
				X earth velocity: %.6f,\tY earth velocity: %.6f,\tZ earth velocity: %.6f,\tAirspeed: %.6f\n\
				X body acceleration: %.6f,\tY body acceleration: %.6f,\tZ body acceleration: %.6f\n\
				Engine rotation speed: %.6f,\tEngine thrust: %.6f\n",
				local_vehicle_global_position.latitude, local_vehicle_global_position.longitude,
				local_vehicle_global_position.altitude, local_vehicle_global_position.ground_level,
				local_vehicle_attitude.roll, local_vehicle_attitude.pitch, local_vehicle_attitude.yaw,
				local_vehicle_attitude.roll_rate, local_vehicle_attitude.pitch_rate, local_vehicle_attitude.yaw_rate,
				local_vehicle_attitude.vx, local_vehicle_attitude.vy, local_vehicle_attitude.vz,
				local_vehicle_global_position.vx, local_vehicle_global_position.vy, local_vehicle_global_position.vz, local_airspeed.indicated_airspeed_m_s,
				local_vehicle_attitude.ax, local_vehicle_attitude.ay, local_vehicle_attitude.az,
				local_vehicle_attitude.engine_rotation_speed, local_vehicle_attitude.thrust);
		fflush (stdout);
		
		// OUTPUTS
		fprintf (stdout, "\nFlight_controls\n\
				Aileron:\t%.6f\n\
				Elevator:\t%.6f\n\
				Rudder:\t\t%.6f\n\
				Throttle:\t%.6f\n",
				local_actuator_controls.aileron,
				local_actuator_controls.elevator,
				local_actuator_controls.rudder,
				local_actuator_controls.throttle);
		fflush (stdout);

		// allow the user to control the console
		release_console_control ();
	}
	

	// ************************************* unsubscribe ******************************************
	if (orb_unsubscribe (ORB_ID(airspeed), airspeed_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to airspeed topic\n");

	if (orb_unsubscribe (ORB_ID(vehicle_global_position), vehicle_global_position_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to vehicle_global_position topic\n");

	if (orb_unsubscribe (ORB_ID(vehicle_attitude), vehicle_attitude_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to vehicle_attitude topic\n");

	if (orb_unsubscribe (ORB_ID(actuator_controls), actuator_controls_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to actuator_controls topic\n");
	
	return 0;
}
