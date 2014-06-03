/*
 * @file attitude_estimator_ideal.c
 *
 */


#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


#include "../../ORB/ORB.h"
#include "../../ORB/topics/vehicle_hil_attitude.h"
#include "../../ORB/topics/vehicle_attitude.h"


/*
 * Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 */
void* attitude_estimator_ideal_thread_main (void* args)
{
	/* welcome user */
	fprintf (stdout, "Attitude estimator started\n");
	fflush(stdout);

	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_hil_attitude_s hil_att;
	memset(&hil_att, 0, sizeof(hil_att));

	/* subscribe to hil attitude */
	orb_subscr_t sub_hil_att = orb_subscribe(ORB_ID(vehicle_hil_attitude));
	if (sub_hil_att == -1)
	{
		fprintf (stderr, "Attitude estimator thread failed to subscribe to the vehicle_hil_attitude topic\n");
		exit (-1);
	}

	/* advertise attitude */
	orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude));
	if (pub_att == -1)
	{
		fprintf (stderr, "Attitude estimator thread failed to advertise the vehicle_attitude topic\n");
		exit (-1);
	}
	orb_publish (ORB_ID(vehicle_attitude), pub_att, &att);

	int updated;
	absolute_time usec_max_poll_wait_time = 1000000;
	
	
	/* Main loop*/
	while (!_shutdown_all_systems) {
		/* wait for up to 1000ms for data */
		updated = orb_poll(ORB_ID(vehicle_hil_attitude), sub_hil_att, usec_max_poll_wait_time);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (updated < 0)
		{
			fprintf (stderr, "Attitude estimator failed to poll sensors\n");
			continue;
		}
		else if (updated)
		{
			orb_copy(ORB_ID(vehicle_hil_attitude), sub_hil_att, &hil_att);
			memcpy (&att, &hil_att, sizeof(att));
		}


		// Broadcast
		orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
	}


	// *********************************** unsubscribe and unadvertise ****************************************
	orb_unsubscribe (ORB_ID(vehicle_hil_attitude), sub_hil_att, pthread_self());

	orb_unadvertise (ORB_ID(vehicle_attitude), pub_att, pthread_self());

	return 0;
}
