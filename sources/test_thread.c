// test_thread.c

#include <sys/types.h>
#include <pthread.h>
#include <semaphore.h>
#include "test_thread.h"
#include "uav_library/common.h"
#include "uav_library/io_ctrl/comunicator.h"
#include "uav_library/console_controller/console_controller.h"

#include "ORB/ORB.h"
#include "ORB/topics/airspeed.h"

/*
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <signal.h>
#include "comunicator.h"
#include "socket_io.h"
*/


pthread_t test_thread_id, test_thread_id2;


void* test_thread_body (void* args)
{
	absolute_time max_update_interval = 5000000;
	//absolute_time last_publish_time;
	//absolute_time usec_max_poll_wait_time = 1000000;
	//int poll_return_value;
	int wait_return_value;
	struct airspeed_s local_airspeed;
	orb_subscr_t airspeed_sub;

	
	// ********************** Subscribe to the topics ******************************
	airspeed_sub = orb_subscribe (ORB_ID(airspeed));
	if (airspeed_sub < 0)
	{
		fprintf (stderr, "Test thread failed subscribing to airspeed topic\n");
		return 0;
	}
	
	// return -1 if an error occur, 0 otherwise
	if (orb_set_interval (ORB_ID(airspeed), airspeed_sub, max_update_interval) < 0)
	{
		fprintf (stdout, "Failed to set max update interval for airspeed topic\n");
		return 0;
	}
	
	
	while (!_shutdown_all_systems)
	{
		/*
		if (!orb_stat (ORB_ID(airspeed), airspeed_sub, &last_publish_time))
		{
			fprintf (stdout, "Test thread successfully obtain publish_time of airspeed topic - ");
			fprintf (stdout, "last_publish_time: %ld\n", (long int) last_publish_time);
			fflush (stdout);
		}
		
		if (orb_check (ORB_ID(airspeed), airspeed_sub) > 0)
		{
			orb_copy (ORB_ID(airspeed), airspeed_sub, (void *) &local_airspeed);
			fprintf (stdout, "Test thread successfully copy the airspeed after check - ");
			fprintf (stdout, "airspeed: %f\n", local_airspeed.true_airspeed_m_s);
			fflush (stdout);
		}
		*/

		/*
		poll_return_value = orb_poll (ORB_ID(airspeed), airspeed_sub, usec_max_poll_wait_time);
		
		// do not plot until the user has finish to control the console
		get_console_unique_control ();

		if (poll_return_value < 0)
		{
			fprintf (stderr, "Test thread experienced an error waiting for airspeed topic\n");
			break;
		}
		else if (poll_return_value)
		{
			orb_copy (ORB_ID(airspeed), airspeed_sub, (void *) &local_airspeed);
			fprintf (stdout, "Test thread successfully copy the airspeed after poll - ");
			fprintf (stdout, "airspeed: %f\n", local_airspeed.true_airspeed_m_s);
			fflush (stdout);
		}
		else
		{
			fprintf (stdout, "Test thread experienced a timeout waiting for airspeed topic\n");
			fflush (stdout);
		}
		
		// allow the user to control the console
		release_console_control ();
		*/

		wait_return_value = orb_wait (ORB_ID(airspeed), airspeed_sub);
		if (wait_return_value < 0)
		{
			fprintf (stderr, "Test thread experienced an error waiting for airspeed topic\n");
			break;
		}
		else if (wait_return_value)
		{
			orb_copy (ORB_ID(airspeed), airspeed_sub, (void *) &local_airspeed);

			// do not plot until the user has finish to control the console
			get_console_unique_control ();

			fprintf (stdout, "Test thread successfully copy the airspeed after wait - ");
			fprintf (stdout, "airspeed: %f\n", local_airspeed.true_airspeed_m_s);
			fflush (stdout);

			// allow the user to control the console
			release_console_control ();
		}
	}
	
	// **************************************** unsubscribe ******************************************
	if (orb_unsubscribe (ORB_ID(airspeed), airspeed_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to mission topic\n");
	
	return 0;
}
