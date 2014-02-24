// autopilot.c

#include <pthread.h>

#include "autopilot.h"
#include "commander.h"
#include "autopilot_parameters.h"
#include "../uav_library/common.h"
#include "../uav_library/param/param.h"

#include "../ORB/ORB.h"
#include "../ORB/topics/airspeed.h"
#include "../ORB/topics/vehicle_attitude.h"
#include "../ORB/topics/position/vehicle_global_position.h"
#include "../ORB/topics/actuator/actuator_controls.h"

pthread_t autopilot_thread_id;
int autopilot_thread_return_value = 0;

/* Subscribe to airspeed, vehicle_global_position, vehicle_attitude topic */
orb_subscr_t airspeed_sub;
orb_subscr_t vehicle_global_position_sub;
orb_subscr_t vehicle_attitude_sub;
struct airspeed_s local_airspeed;
struct vehicle_global_position_s local_vehicle_global_position;
struct vehicle_attitude_s local_vehicle_attitude;
/* Advertise actuator_controls topic */
orb_advert_t actuator_controls_adv;
struct actuator_controls_s local_actuator_controls;


int compute_flight_controls ()
{
	//Proportional control for roll and pitch
	local_actuator_controls.aileron = -0.05*(local_vehicle_attitude.roll*57.2957795 - 0);
	local_actuator_controls.elevator = 0.1*(local_vehicle_attitude.pitch*57.2957795 - 5);
	local_actuator_controls.rudder = 0;
	local_actuator_controls.throttle = 0.95;

	//Limit control inputs
	if (fabs(local_actuator_controls.aileron) > 0.6)
		local_actuator_controls.aileron = (local_actuator_controls.aileron/fabs(local_actuator_controls.aileron))*0.6;
	if (fabs(local_actuator_controls.elevator) > 0.6)
		local_actuator_controls.elevator = (local_actuator_controls.elevator/fabs(local_actuator_controls.elevator))*0.6;
	
	return 0;
}


void* autopilot_loop (void* args)
{
	int wait_return_value;
	pthread_t commander_thread_id;

	// start commander
	if (pthread_create (&commander_thread_id, NULL, commander_thread_main, NULL) != 0)
	{
		fprintf (stderr, "Can't create the commander thread (errno: %d)\n", errno);
		exit(-1);
	}


	// XXX MAYBE TO MOVE
	param_define_int("BAT_V_EMPTY", 3);
	param_define_int("BAT_V_FULL", 6);
	param_define_int("BAT_N_CELLS", 4);

	param_define_float("TAKEOFF_ALT", 10);
	param_define_float("LAND_TIME", 4.0f);
	param_define_float("LAND_ALT", 2.0f); // XXX check this value
	param_define_float("LAND_THRUST", 0.25f);


	// ************************************* subscribe and advertise ******************************************
	airspeed_sub = orb_subscribe (ORB_ID(airspeed));
	if (airspeed_sub == -1)
	{
		fprintf (stderr, "Autopilot thread can't subscribe to the airspeed topic\n");
		exit (-1);
	}
	
	vehicle_global_position_sub = orb_subscribe (ORB_ID(vehicle_global_position));
	if (vehicle_global_position_sub == -1)
	{
		fprintf (stderr, "Autopilot thread can't subscribe to the vehicle_global_position topic\n");
		exit (-1);
	}

	vehicle_attitude_sub = orb_subscribe (ORB_ID(vehicle_attitude));
	if (vehicle_attitude_sub == -1)
	{
		fprintf (stderr, "Autopilot thread can't subscribe to the vehicle_attitude topic\n");
		exit (-1);
	}
	
	actuator_controls_adv = orb_advertise (ORB_ID(actuator_controls));
	if (actuator_controls_adv == -1)
	{
		fprintf (stderr, "Autopilot thread can't advertise the vehicle_attitude topic\n");
		exit (-1);
	}
	
	
	// **************************************** start autopilot loop ******************************************
	while (!_shutdown_all_systems)
	{
		/*
		wait_return_value = orb_wait (ORB_ID(airspeed), airspeed_sub);
		if (wait_return_value < 0)
		{
			//if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Autopilot thread experienced an error waiting for airspeed topic\n");
				autopilot_thread_return_value = -1;
			}
			
			break;
		}
		else if (wait_return_value)
			orb_copy (ORB_ID(airspeed), airspeed_sub, (void *) &local_airspeed);
		
		wait_return_value = orb_wait (ORB_ID(vehicle_global_position), vehicle_global_position_sub);
		if (wait_return_value < 0)
		{
			//if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Autopilot thread experienced an error waiting for vehicle_global_position topic\n");
				autopilot_thread_return_value = -1;
			}
			
			break;
		}
		else if (wait_return_value)
			orb_copy (ORB_ID(vehicle_global_position), vehicle_global_position_sub, (void *) &local_vehicle_global_position);
		*/
		
		wait_return_value = orb_wait (ORB_ID(vehicle_attitude), vehicle_attitude_sub);
		if (wait_return_value < 0)
		{
			//if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Autopilot thread experienced an error waiting for vehicle_attitude topic\n");
				autopilot_thread_return_value = -1;
			}
			
			break;
		}
		else if (wait_return_value)
			orb_copy (ORB_ID(vehicle_attitude), vehicle_attitude_sub, (void *) &local_vehicle_attitude);
		

		// AUTOPILOT LOGIC
		if (compute_flight_controls () < 0)
		{
			//if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Can't compute flight controls\n");
				autopilot_thread_return_value = -1;
			}
			
			break;
		}
		
		// **************************************** publish flight_controls ******************************************
		if (orb_publish (ORB_ID(actuator_controls), actuator_controls_adv, (const void *) &local_actuator_controls) < 0)
		{
			//if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Failed to publish the actuator_controls topic\n");
				autopilot_thread_return_value = -1;
			}
			
			break;
		}
	}
	
	
	// *********************************** unsubscribe and unadvertise ****************************************
	if (orb_unsubscribe (ORB_ID(airspeed), airspeed_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to airspeed topic\n");

	if (orb_unsubscribe (ORB_ID(vehicle_global_position), vehicle_global_position_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to vehicle_global_position topic\n");

	if (orb_unsubscribe (ORB_ID(vehicle_attitude), vehicle_attitude_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to vehicle_attitude topic\n");

	if (orb_unadvertise (ORB_ID(actuator_controls), actuator_controls_adv, pthread_self()) < 0)
		fprintf (stderr, "Failed to unadvertise the actuator_controls topic\n");
	
	
	pthread_join (commander_thread_id, NULL);

	pthread_exit(&autopilot_thread_return_value);
	return 0;
}

