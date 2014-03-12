/**
 * @file fw_attitude_control_main.c
 *
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <time.h>

#include "../../ORB/ORB.h"
#include "../../ORB/topics/sensors/sensor_airspeed.h"
#include "../../ORB/topics/sensors/sensor_accel.h"
#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/vehicle_control_flags.h"
#include "../../ORB/topics/parameter_update.h"

#include "../../uav_library/common.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/math/limits.h"

#include "ECL_pitch_controller.h"
#include "ECL_roll_controller.h"
#include "ECL_yaw_controller.h"
#include "fw_attitude_control_main.h"
#include "fw_attitude_control_params.h"


static orb_subscr_t _att_sub = -1;				/**< vehicle attitude subscription */
static orb_subscr_t _accel_sub = -1;				/**< accelerometer subscription */
static orb_subscr_t _att_sp_sub = -1;			/**< vehicle attitude setpoint */
static orb_subscr_t _airspeed_sub = -1;			/**< airspeed subscription */
static orb_subscr_t _vcontrol_flags_sub = -1;		/**< vehicle status subscription */
static orb_subscr_t _params_sub = -1;			/**< notification of parameter updates */
static orb_subscr_t _manual_sub = -1;			/**< notification of manual control updates */

static orb_advert_t _rate_sp_adv = -1;			/**< rate setpoint publication */
static orb_advert_t _attitude_sp_adv = -1;		/**< attitude setpoint point */
static orb_advert_t _actuators_adv = -1;		/**< actuator control group 0 setpoint */

static struct vehicle_attitude_s _att;					/**< vehicle attitude */
static struct sensor_accel_s _accel;						/**< body frame accelerations */
static struct vehicle_attitude_setpoint_s _att_sp;		/**< vehicle attitude setpoint */
static struct manual_control_setpoint_s _manual;		/**< r/c channel data */
static struct sensor_airspeed_s _airspeed;					/**< airspeed */
static struct vehicle_control_flags_s _vcontrol_flags;	/**< vehicle control mode */

static bool_t _setpoint_valid = 0;		/**< flag if the position control setpoint is valid */
static bool_t _airspeed_valid = 0;		/**< flag if the airspeed measurement is valid */

static absolute_time last_run = 0;



static void vehicle_control_flags_poll()
{
	/* Check HIL state if vehicle status has changed */
	bool_t vcontrol_mode_updated = orb_check(ORB_ID(vehicle_control_flags), _vcontrol_flags_sub);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_flags), _vcontrol_flags_sub, &_vcontrol_flags);
	}
}

static void vehicle_manual_poll()
{
	/* get pilots inputs */
	bool_t manual_updated = orb_check(ORB_ID(manual_control_setpoint), _manual_sub);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

static bool_t vehicle_airspeed_poll()
{
	/* check if there is a new position */
	bool_t airspeed_updated = orb_check(ORB_ID(sensor_airspeed), _airspeed_sub);

	if (airspeed_updated) {
		orb_copy(ORB_ID(sensor_airspeed), _airspeed_sub, &_airspeed);
		return 1 /* true */;
	}

	return 0 /* false */;
}

static void vehicle_accel_poll()
{
	/* check if there is a new position */
	bool_t accel_updated = orb_check(ORB_ID(sensor_accel), _accel_sub);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

static void vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool_t att_sp_updated = orb_check(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = 1 /* true */;
	}
}


void* fw_attitude_control_thread_main (void* args)
{
	/* welcome user */
	fprintf (stdout, "Attitude controller started\n");
	fflush(stdout);


	bool_t lock_integrator;
	float deltaT;
	float roll_sp, pitch_sp, throttle_sp = 0.0f;
	float roll_rad, pitch_rad, yaw_rad;
	float airspeed, airspeed_scaling;	/* scale around tuning airspeed */
	const float actuator_scaling = 1.0f / (M_PI / 4.0f);	/* scale from radians to normalized -1 .. 1 range */

	int updated;
	absolute_time usec_max_poll_wait_time = 200000;

	/* declare and safely initialize all structs */
	struct parameter_update_s p_update;
	memset(&p_update, 0, sizeof(p_update));
	memset(&_att, 0, sizeof(_att));					/**< vehicle attitude */
	memset(&_accel, 0, sizeof(_accel));						/**< body frame accelerations */
	memset(&_att_sp, 0, sizeof(_att_sp));		/**< vehicle attitude setpoint */
	memset(&_manual, 0, sizeof(_manual));		/**< r/c channel data */
	memset(&_airspeed, 0, sizeof(_airspeed));					/**< airspeed */
	memset(&_vcontrol_flags, 0, sizeof(_vcontrol_flags));	/**< vehicle control mode */

	/* output structs */
	struct vehicle_attitude_setpoint_s att_sp;
	struct vehicle_rates_setpoint_s rates_sp;
	struct actuator_controls_s _actuators;
	memset(&att_sp, 0, sizeof(att_sp));
	memset(&rates_sp, 0, sizeof(rates_sp));
	memset(&_actuators, 0, sizeof(_actuators));


	/* abort on a nonzero return value from the parameter init */
	if (fw_attitude_control_param_init() != 0) {
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Attitude controller aborting on startup due to an error\n");
		exit(-1);
	}

	ECL_roll_controller_init ();
	ECL_pitch_controller_init ();
	ECL_yaw_controller_init ();

	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	if (_att_sp_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_attitude_setpoint topic\n");
		exit(-1);
	}

	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	if (_att_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_attitude topic\n");
		exit(-1);
	}

	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	if (_accel_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to sensor_accel topic\n");
		exit(-1);
	}

	_airspeed_sub = orb_subscribe(ORB_ID(sensor_airspeed));
	if (_airspeed_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to airspeed topic\n");
		exit(-1);
	}

	_vcontrol_flags_sub = orb_subscribe(ORB_ID(vehicle_control_flags));
	if (_vcontrol_flags_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to vehicle_control_flags topic\n");
		exit(-1);
	}

	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	if (_params_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to parameter_update topic\n");
		exit(-1);
	}

	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	if (_manual_sub < 0)
	{
		fprintf (stderr, "Attitude controller thread failed to subscribe to manual_control_setpoint topic\n");
		exit(-1);
	}

	/*
	 * do advertises
	 */
	_rate_sp_adv = orb_advertise(ORB_ID(vehicle_rates_setpoint));
	if (_rate_sp_adv < 0) {
		fprintf (stderr, "Attitude controller thread failed to advertise vehicle_rates_setpoint topic\n");
		exit(-1);
	}

	_attitude_sp_adv = orb_advertise(ORB_ID(vehicle_attitude_setpoint));
	if (_attitude_sp_adv < 0) {
		fprintf (stderr, "Attitude controller thread failed to advertise vehicle_attitude_setpoint topic\n");
		exit(-1);
	}

	_actuators_adv = orb_advertise(ORB_ID(actuator_controls));
	if (_actuators_adv < 0) {
		fprintf (stderr, "Attitude controller thread failed to advertise actuator_controls topic\n");
		exit(-1);
	}

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(ORB_ID(vehicle_control_flags), _vcontrol_flags_sub, 200000 /* usec */);
	orb_set_interval(ORB_ID(vehicle_attitude), _att_sub, 100000 /* usec */);


	/* get an initial update for all sensor and status data */
	(void)vehicle_airspeed_poll();
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_flags_poll();
	vehicle_manual_poll();


	while (!_shutdown_all_systems) {
		updated = orb_check(ORB_ID(parameter_update), _params_sub);
		if (updated) {
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(parameter_update), _params_sub, &p_update);

			/* update parameters from storage */
			fw_attitude_control_parameters_update();
		}

		/* wait for up to 200ms for data */
		updated = orb_poll(ORB_ID(vehicle_attitude), _att_sub, usec_max_poll_wait_time);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (!updated)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (updated < 0) {
			fprintf (stderr, "Attitude controller failed to poll vehicle attitude\n");
			continue;
		}


		/* only run controller if attitude changed */
		deltaT = (get_absolute_time() - last_run) / 1000000.0f;
		last_run = get_absolute_time();

		/* guard against too large deltaT's */
		if (deltaT > 1.0f)
			deltaT = 0.01f;

		/* load local copies */
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

		_airspeed_valid = vehicle_airspeed_poll();
		vehicle_setpoint_poll();
		vehicle_accel_poll();
		vehicle_control_flags_poll();
		vehicle_manual_poll();

		/* lock integrator until control is started */
		lock_integrator = (_vcontrol_flags.flag_control_attitude_enabled)? 0 /* false */ : 1 /* true */;

		/* decide if in stabilized or full manual control */

		if (_vcontrol_flags.flag_control_attitude_enabled) {

			/* if airspeed is smaller than min, the sensor is not giving good readings */
			if (!_airspeed_valid) {
				airspeed = _fw_attitude_control_parameters.airspeed_min + (_fw_attitude_control_parameters.airspeed_max - _fw_attitude_control_parameters.airspeed_min) / 2.0f;
			}
			else if (_airspeed.indicated_airspeed_m_s < _fw_attitude_control_parameters.airspeed_min) {
				airspeed = _fw_attitude_control_parameters.airspeed_min;
			}
			else if (_airspeed.indicated_airspeed_m_s > _fw_attitude_control_parameters.airspeed_max) {
				airspeed = _fw_attitude_control_parameters.airspeed_max;
			}
			else {
				airspeed = _airspeed.indicated_airspeed_m_s;
			}

			airspeed_scaling = _fw_attitude_control_parameters.airspeed_trim / airspeed;
			//warnx("aspd scale: %6.2f act scale: %6.2f", airspeed_scaling, actuator_scaling);

			if (_vcontrol_flags.flag_control_velocity_enabled || _vcontrol_flags.flag_control_position_enabled) {
				roll_sp = _att_sp.roll_body;
				pitch_sp = _att_sp.pitch_body;
				throttle_sp = _att_sp.thrust;

				/* reset integrals where needed */
				if (_att_sp.roll_reset_integral)
					ECL_roll_controller_reset_integrator();

			} else {
				/*
				 * Scale down roll and pitch as the setpoints are radians
				 * and a typical remote can only do 45 degrees, the mapping is
				 * -1..+1 to -45..+45 degrees or -0.75..+0.75 radians.
				 *
				 * With this mapping the stick angle is a 1:1 representation of
				 * the commanded attitude. If more than 45 degrees are desired,
				 * a scaling parameter can be applied to the remote.
				 */
				roll_sp = _manual.roll * 0.75f;
				pitch_sp = _manual.pitch * 0.75f;
				throttle_sp = _manual.throttle;
				_actuators.flaps = _manual.flaps;

				/*
				 * in manual mode no external source should / does emit attitude setpoints.
				 * emit the manual setpoint here to allow attitude controller tuning
				 * in attitude control mode.
				 */
				att_sp.roll_body = roll_sp;
				att_sp.pitch_body = pitch_sp;
				att_sp.yaw_body = 0.0f;
				att_sp.thrust = throttle_sp;

				/* lazily publish the setpoint only once available */
				if (_attitude_sp_adv > 0) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_adv, &att_sp);

				} else {

				}
			}

			roll_rad = ECL_roll_controller_control (roll_sp, _att.roll, _att.roll_rate, airspeed_scaling, lock_integrator, _fw_attitude_control_parameters.airspeed_min, _fw_attitude_control_parameters.airspeed_max, airspeed);
			_actuators.aileron = (check_finite(roll_rad))? roll_rad * actuator_scaling : 0.0f;

			pitch_rad = ECL_pitch_controller_control (pitch_sp, _att.pitch, _att.pitch_rate, _att.roll, airspeed_scaling, lock_integrator, _fw_attitude_control_parameters.airspeed_min, _fw_attitude_control_parameters.airspeed_max, airspeed);
			_actuators.elevator = (check_finite(pitch_rad))? pitch_rad * actuator_scaling : 0.0f;

			yaw_rad = ECL_yaw_controller_control (_att.roll, _att.yaw_rate, _accel.y, airspeed_scaling, lock_integrator, _fw_attitude_control_parameters.airspeed_min, _fw_attitude_control_parameters.airspeed_max, airspeed);
			_actuators.rudder = (check_finite(yaw_rad))? yaw_rad * actuator_scaling : 0.0f;

			/* throttle passed through */
			_actuators.throttle = (check_finite(throttle_sp))? throttle_sp : 0.0f;


			// warnx("aspd: %s: %6.2f, aspd scaling: %6.2f, controls: %5.2f %5.2f %5.2f %5.2f", (_airspeed_valid) ? "valid" : "unknown",
			// 			airspeed, airspeed_scaling, _actuators.aileron, _actuators.elevator,
			// 			_actuators.rudder, _actuators.throttle);


			/*
			 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
			 * only once available
			 */
			rates_sp.roll = ECL_roll_controller_get_desired_rate();
			rates_sp.pitch = ECL_pitch_controller_get_desired_rate();
			rates_sp.yaw = 0.0f; // XXX not yet implemented

			/* publish the attitude setpoint */
			orb_publish(ORB_ID(vehicle_rates_setpoint), _rate_sp_adv, &rates_sp);

		} else {
			/* manual/direct control */
			_actuators.aileron = _manual.roll;
			_actuators.elevator = _manual.pitch;
			_actuators.rudder = _manual.yaw;
			_actuators.throttle = _manual.throttle;
			_actuators.flaps = _manual.flaps;
		}

		/* publish the attitude setpoint */
		orb_publish(ORB_ID(actuator_controls), _actuators_adv, &_actuators);

	}

	/*
	 * do unsubscriptions
	 */
	orb_unsubscribe(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude), _att_sub, pthread_self());
	orb_unsubscribe(ORB_ID(sensor_accel), _accel_sub, pthread_self());
	orb_unsubscribe(ORB_ID(sensor_airspeed), _airspeed_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_control_flags), _vcontrol_flags_sub, pthread_self());
	orb_unsubscribe(ORB_ID(parameter_update), _params_sub, pthread_self());
	orb_unsubscribe(ORB_ID(manual_control_setpoint), _manual_sub, pthread_self());

	/*
	 * do unadvertises
	 */
	orb_unadvertise(ORB_ID(vehicle_rates_setpoint), _rate_sp_adv, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_adv, pthread_self());
	orb_unadvertise(ORB_ID(actuator_controls), _actuators_adv, pthread_self());

	return 0;
}
