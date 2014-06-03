/**
 * @file multirotor_attitude_control_main.c
 *
 * Implementation of multirotor attitude control main loop.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

#include "../../ORB/ORB.h"
#include "../../ORB/topics/vehicle_control_flags.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
#include "../../ORB/topics/setpoint/offboard_control_setpoint.h"
#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
#include "../../ORB/topics/vehicle_status.h"
#include "../../ORB/topics/sensors/sensor_combined.h"
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/parameter_update.h"

#include "../../uav_library/common.h"
#include "../../uav_library/pid/pid.h"
#include "../../uav_library/time/drv_time.h"

#include "multirotor_attitude_control.h"
#include "multirotor_rate_control.h"


//static bool_t motor_test_mode = 0 /* false */;
//static const float min_takeoff_throttle = 0.3f;
static const float yaw_deadzone = 0.01f;



void *multirotor_attitude_control_thread_main()
{
	/* welcome user */
	fprintf (stdout, "Multirotor attitude controller started\n");
	fflush(stdout);

	int i;
	bool_t updated, rates_sp_updated, reset_integral;
	float rates[3];

	/* store last control mode to detect mode switches */
	bool_t control_yaw_position = 1 /* true */;
	bool_t reset_yaw_sp = 1 /* true */;

	/* declare and safely initialize all structs */
	struct parameter_update_s update;
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct offboard_control_setpoint_s offboard_sp;
	memset(&offboard_sp, 0, sizeof(offboard_sp));
	struct vehicle_control_flags_s control_flags;
	memset(&control_flags, 0, sizeof(control_flags));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_status_s status;
	memset(&status, 0, sizeof(status));
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));


	/* subscribe */
	orb_subscr_t vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	if (vehicle_attitude_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_attitude topic\n");
		exit(-1);
	}

	orb_subscr_t parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	if (parameter_update_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to parameter_update topic\n");
		exit(-1);
	}


	orb_subscr_t vehicle_attitude_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	if (vehicle_attitude_setpoint_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_attitude_setpoint topic\n");
		exit(-1);
	}


	orb_subscr_t offboard_control_setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
	if (offboard_control_setpoint_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to offboard_control_setpoint topic\n");
		exit(-1);
	}


	orb_subscr_t vehicle_control_flags_sub = orb_subscribe(ORB_ID(vehicle_control_flags));
	if (vehicle_control_flags_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_control_flags topic\n");
		exit(-1);
	}


	orb_subscr_t manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	if (manual_control_setpoint_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to manual_control_setpoint topic\n");
		exit(-1);
	}


	orb_subscr_t sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	if (sensor_combined_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to sensor_combined topic\n");
		exit(-1);
	}


	orb_subscr_t vehicle_rates_setpoint_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	if (vehicle_rates_setpoint_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_rates_setpoint topic\n");
		exit(-1);
	}


	orb_subscr_t vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	if (vehicle_status_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_status topic\n");
		exit(-1);
	}


	/* publish actuator controls */
	for (i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	if (actuator_pub == -1)
	{
		fprintf (stderr, "Comunicator thread failed to advertise the actutor_controls topic\n");
		exit (-1);
	}
	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint));
	if (att_sp_pub == -1)
	{
		fprintf (stderr, "Comunicator thread failed to advertise the vehicle_attitude_setpoint topic\n");
		exit (-1);
	}
	orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

	orb_advert_t rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint));
	if (rates_sp_pub == -1)
	{
		fprintf (stderr, "Comunicator thread failed to advertise the vehicle_rates_setpoint topic\n");
		exit (-1);
	}
	orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);



	while (!_shutdown_all_systems) {
		/* wait for a sensor update, check for exit condition every 500 ms */
		updated = orb_poll(ORB_ID(vehicle_attitude), vehicle_attitude_sub, 500000);

		/* timed out - periodic check for _shutdown_all_systems, etc. */
		if (!updated)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (updated < 0) {
			fprintf (stderr, "Attitude controller failed to poll vehicle attitude\n");
			continue;
		}

		/* only run controller if attitude changed */
		/* attitude */
		orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);


		/* parameters */
		updated = orb_check (ORB_ID(parameter_update), parameter_update_sub);
		if (updated) {
			orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
			/* update parameters */
		}

		/* control mode */
		updated = orb_check (ORB_ID(vehicle_control_flags), vehicle_control_flags_sub);
		if (updated) {
			orb_copy(ORB_ID(vehicle_control_flags), vehicle_control_flags_sub, &control_flags);
		}

		/* manual control setpoint */
		updated = orb_check (ORB_ID(manual_control_setpoint), manual_control_setpoint_sub);
		if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
		}

		/* attitude setpoint */
		updated = orb_check (ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub);
		if (updated) {
			orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub, &att_sp);
		}

		/* offboard control setpoint */
		updated = orb_check (ORB_ID(offboard_control_setpoint), offboard_control_setpoint_sub);
		if (updated) {
			orb_copy(ORB_ID(offboard_control_setpoint), offboard_control_setpoint_sub, &offboard_sp);
		}

		/* vehicle status */
		updated = orb_check (ORB_ID(vehicle_status), vehicle_status_sub);
		if (updated) {
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &status);
		}

		/* sensors */
		updated = orb_check (ORB_ID(sensor_combined), sensor_combined_sub);
		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);
		}

		/* set flag to safe value */
		control_yaw_position = 1 /* true */;

		/* reset yaw setpoint if not armed */
		if (!control_flags.flag_armed) {
			reset_yaw_sp = 1 /* true */;
		}

		/* define which input is the dominating control input */
		if (control_flags.flag_control_offboard_enabled) {
			/* offboard inputs */
			if (offboard_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_RATES) {
				rates_sp.roll = offboard_sp.p1;
				rates_sp.pitch = offboard_sp.p2;
				rates_sp.yaw = offboard_sp.p3;
				rates_sp.thrust = offboard_sp.p4;
				rates_sp.timestamp = get_absolute_time();
				orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);

			} else if (offboard_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE) {
				att_sp.roll_body = offboard_sp.p1;
				att_sp.pitch_body = offboard_sp.p2;
				att_sp.yaw_body = offboard_sp.p3;
				att_sp.thrust = offboard_sp.p4;
				att_sp.timestamp = get_absolute_time();
				/* publish the result to the vehicle actuators */
				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
			}

			/* reset yaw setpoint after offboard control */
			reset_yaw_sp = 1 /* true */;

		} else if (control_flags.flag_control_manual_enabled) {
			/* manual input */
			if (control_flags.flag_control_attitude_enabled) {
				/* control attitude, update attitude setpoint depending on mode */
				if (att_sp.thrust < 0.1f) {
					/* no thrust, don't try to control yaw */
					rates_sp.yaw = 0.0f;
					control_yaw_position = 0 /* false */;

					if (status.condition_landed) {
						/* reset yaw setpoint if on ground */
						reset_yaw_sp = 1 /* true */;
					}

				} else {
					/* only move yaw setpoint if manual input is != 0 */
					if (manual.yaw < -yaw_deadzone || yaw_deadzone < manual.yaw) {
						/* control yaw rate */
						control_yaw_position = 0 /* false */;
						rates_sp.yaw = manual.yaw;
						reset_yaw_sp = 1 /* true */;	// has no effect on control, just for beautiful log

					} else {
						control_yaw_position = 1 /* true */;
					}
				}

				if (!control_flags.flag_control_velocity_enabled) {
					/* update attitude setpoint if not in position control mode */
					att_sp.roll_body = manual.roll;
					att_sp.pitch_body = manual.pitch;

					if (!control_flags.flag_control_climb_rate_enabled) {
						/* pass throttle directly if not in altitude control mode */
						att_sp.thrust = manual.thrust;
					}
				}

				/* reset yaw setpint to current position if needed */
				if (reset_yaw_sp) {
					att_sp.yaw_body = att.yaw;
					reset_yaw_sp = 0 /* false */;
				}

				/*if (motor_test_mode) {
					printf("testmode");
					att_sp.roll_body = 0.0f;
					att_sp.pitch_body = 0.0f;
					att_sp.yaw_body = 0.0f;
					att_sp.thrust = 0.1f;
				}*/

				att_sp.timestamp = get_absolute_time();

				/* publish the attitude setpoint */
				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

			} else {
				/* manual rate inputs (ACRO), from RC control or joystick */
				if (control_flags.flag_control_rates_enabled) {
					rates_sp.roll = manual.roll;
					rates_sp.pitch = manual.pitch;
					rates_sp.yaw = manual.yaw;
					rates_sp.thrust = manual.thrust;
					rates_sp.timestamp = get_absolute_time();
				}

				/* reset yaw setpoint after ACRO */
				reset_yaw_sp = 1 /* true */;
			}

		} else {
			if (!control_flags.flag_control_auto_enabled) {
				/* no control, try to stay on place */
				if (!control_flags.flag_control_velocity_enabled) {
					/* no velocity control, reset attitude setpoint */
					att_sp.roll_body = 0.0f;
					att_sp.pitch_body = 0.0f;
					att_sp.timestamp = get_absolute_time();

					orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
				}
			}

			/* reset yaw setpoint after non-manual control */
			reset_yaw_sp = 1 /* true */;
		}

		/* check if we should we reset integrals */
		reset_integral = !control_flags.flag_armed || att_sp.thrust < 0.1f;	// TODO use landed status instead of throttle

		/* run attitude controller if needed */
		if (control_flags.flag_control_attitude_enabled) {
			multirotor_control_attitude(&att_sp, &att, &rates_sp, control_yaw_position, reset_integral);

			orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);
		}

		/* run rates controller if needed */
		if (control_flags.flag_control_rates_enabled) {
			/* get current rate setpoint */
			rates_sp_updated = orb_check (ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub);
			if (rates_sp_updated) {
				orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub, &rates_sp);
			}

			/* apply controller */
			rates[0] = att.roll_rate;
			rates[1] = att.pitch_rate;
			rates[2] = att.yaw_rate;

			multirotor_control_rates(&rates_sp, rates, &actuators, reset_integral);

		} else {
			/* rates controller disabled, set actuators to zero for safety */
			actuators.control[0] = 0.0f;
			actuators.control[1] = 0.0f;
			actuators.control[2] = 0.0f;
			actuators.control[3] = 0.0f;
		}

		/* fill in manual control values */
		actuators.control[4] = manual.flaps;
		//actuators.control[5] = manual.aux1;
		//actuators.control[6] = manual.aux2;
		//actuators.control[7] = manual.aux3;

		orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
	}

	fprintf (stdout, "INFO: Attitude controller exiting, disarming motors\n");
	fflush (stdout);

	/* kill all outputs */
	for (i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	/*
	 * do unsubscriptions
	 */
	orb_unsubscribe(ORB_ID(vehicle_attitude), vehicle_attitude_sub, pthread_self());
	orb_unsubscribe(ORB_ID(parameter_update), parameter_update_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub, pthread_self());
	orb_unsubscribe(ORB_ID(offboard_control_setpoint), offboard_control_setpoint_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_control_flags), vehicle_control_flags_sub, pthread_self());
	orb_unsubscribe(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, pthread_self());
	orb_unsubscribe(ORB_ID(sensor_combined), sensor_combined_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_status), vehicle_status_sub, pthread_self());

	/*
	 * do unadvertises
	 */
	orb_unadvertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, pthread_self());


	return 0;
}

