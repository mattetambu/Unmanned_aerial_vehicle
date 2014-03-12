/**
 * @file fw_position_control_main.c
 * Implementation of a generic position controller based on the L1 norm. Outputs a bank / roll
 * angle, equivalent to a lateral motion (for copters and rovers).
 *
 * Original publication for horizontal control class:
 *	S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
 *	Proceedings of the AIAA Guidance, Navigation and Control
 *	Conference, Aug 2004. AIAA-2004-4900.
 *
 * Original implementation for total energy control class:
 *	Paul Riseborough and Andrew Tridgell, 2013 (code in lib/external_lgpl)
 *
 * More details and acknowledgements in the referenced library headers.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <time.h>

#include "../../ORB/ORB.h"
#include "../../ORB/topics/sensors/sensor_airspeed.h"
#include "../../ORB/topics/sensors/sensor_accel.h"
#include "../../ORB/topics/position/vehicle_global_position.h"
#include "../../ORB/topics/setpoint/vehicle_global_position_set_triplet.h"
#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/vehicle_control_flags.h"
#include "../../ORB/topics/navigation_capabilities.h"
#include "../../ORB/topics/parameter_update.h"

#include "../../uav_library/common.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/math/limits.h"
#include "../../uav_library/math/Vector.h"
#include "../../uav_library/math/Vector2f.h"
#include "../../uav_library/math/Vector3f.h"
#include "../../uav_library/math/Matrix.h"
#include "../../uav_library/math/Dcm.h"

#include "ECL_l1_position_controller.h"
#include "ECL_tecs.h"
#include "fw_position_control_main.h"
#include "fw_position_control_params.h"


static orb_subscr_t _global_pos_sub = -1;
static orb_subscr_t _global_set_triplet_sub = -1;
static orb_subscr_t _att_sub = -1;				/**< vehicle attitude subscription */
static orb_subscr_t _airspeed_sub = -1;			/**< airspeed subscription */
static orb_subscr_t _control_flags_sub = -1;	/**< vehicle status subscription */
static orb_subscr_t _params_sub = -1;			/**< notification of parameter updates */
static orb_subscr_t _manual_control_sub = -1;	/**< notification of manual control updates */
static orb_subscr_t _accel_sub = -1;			/**< body frame accelerations */

static orb_advert_t _attitude_sp_adv = -1;			/**< attitude setpoint */
static orb_advert_t _nav_capabilities_adv = -1;		/**< navigation capabilities publication */

static struct vehicle_attitude_s _att;						/**< vehicle attitude */
static struct vehicle_attitude_setpoint_s _att_sp;			/**< vehicle attitude setpoint */
static struct navigation_capabilities_s _nav_capabilities;	/**< navigation capabilities */
static struct manual_control_setpoint_s _manual;			/**< r/c channel data */
static struct sensor_airspeed_s _airspeed;						/**< airspeed */
static struct vehicle_control_flags_s _control_flags;		/**< vehicle status */
static struct vehicle_global_position_s _global_pos;		/**< global vehicle position */
static struct vehicle_global_position_set_triplet_s _global_triplet;	/**< triplet of global setpoints */
static struct sensor_accel_s _accel;							/**< body frame accelerations */

static bool_t _setpoint_valid = 0;		/**< flag if the position control setpoint is valid */

/** manual control states */
static float _seatbelt_hold_heading;		/**< heading the system should hold in seatbelt mode */
static float _loiter_hold_lat;
static float _loiter_hold_lon;
static float _loiter_hold_alt;
static bool_t _loiter_hold = 0;

static float _launch_lat;
static float _launch_lon;
static float _launch_alt;
static bool_t _launch_valid;

/* land states */
/* not in non-abort mode for landing yet */
static bool_t land_noreturn = 0;
/* heading hold */
static float target_bearing;

/* throttle and airspeed states */
static float _airspeed_error = 0;				///< airspeed error to setpoint in m/s
static bool_t _airspeed_valid = 0;				///< flag if a valid airspeed estimate exists
uint64_t _airspeed_last_valid;			///< last time airspeed was valid. Used to detect sensor failures
static float _groundspeed_undershoot = 0;		///< ground speed error to min. speed in m/s
static bool_t _global_pos_valid = 0;				///< global position is valid
static Dcm _R_nb;				///< current attitude

static absolute_time last_run = 0;




static void vehicle_control_mode_poll()
{
	/* Check HIL state if vehicle status has changed */
	bool_t vstatus_updated = orb_check(ORB_ID(vehicle_control_flags), _control_flags_sub);

	if (vstatus_updated) {
		bool_t was_armed = _control_flags.flag_armed;

		orb_copy(ORB_ID(vehicle_control_flags), _control_flags_sub, &_control_flags);
		if (!was_armed && _control_flags.flag_armed) {
			_launch_lat = _global_pos.latitude;
			_launch_lon = _global_pos.longitude;
			_launch_alt = _global_pos.altitude;
			_launch_valid = 1 /* true */;
		}
	}
}

static bool_t vehicle_airspeed_poll()
{
	/* check if there is an airspeed update or if it timed out */
	bool_t airspeed_updated = orb_check(ORB_ID(sensor_airspeed), _airspeed_sub);

	if (airspeed_updated) {
		orb_copy(ORB_ID(sensor_airspeed), _airspeed_sub, &_airspeed);
		_airspeed_valid = 1 /* true */;
		_airspeed_last_valid = get_absolute_time();
		return 1 /* true */;

	} else {

		/* no airspeed updates for one second */
		if (_airspeed_valid && (get_absolute_time() - _airspeed_last_valid) > 1e6) {
			_airspeed_valid = 0 /* false */;
		}
	}

	/* update TECS state */
	ECL_tecs_enable_airspeed(_airspeed_valid);

	return 0 /* false */;
}

static int vehicle_attitude_poll()
{
	/* check if there is a new position */
	bool_t att_updated = orb_check(ORB_ID(vehicle_attitude), _att_sub);
	int i, j;

	if (att_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

		/* set rotation matrix */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				MATHLIB_ASSERT (Dcm_set_data (&_R_nb, _att.R[i][j], i, j));
	}

	return 0;
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
	bool_t global_sp_updated = orb_check(ORB_ID(vehicle_global_position_set_triplet), _global_set_triplet_sub);

	if (global_sp_updated) {
		orb_copy(ORB_ID(vehicle_global_position_set_triplet), _global_set_triplet_sub, &_global_triplet);
		_setpoint_valid = 1 /* true */;
	}
}


float calculate_target_airspeed(float airspeed_demand)
{
	float airspeed;

	if (_airspeed_valid) {
		airspeed = _airspeed.true_airspeed_m_s;

	} else {
		airspeed = _fw_position_control_parameters.airspeed_min + (_fw_position_control_parameters.airspeed_max - _fw_position_control_parameters.airspeed_min) / 2.0f;
	}

	/* cruise airspeed for all modes unless modified below */
	float target_airspeed = airspeed_demand;

	/* add minimum ground speed undershoot (only non-zero in presence of sufficient wind) */
	target_airspeed += _groundspeed_undershoot;

	if (0/* throttle nudging enabled */) {
		//target_airspeed += nudge term.
	}

	/* sanity check: limit to range */
	target_airspeed = constrain(target_airspeed, _fw_position_control_parameters.airspeed_min, _fw_position_control_parameters.airspeed_max);


	/* plain airspeed error */
	_airspeed_error = target_airspeed - airspeed;

	return target_airspeed;
}

int calculate_gndspeed_undershoot()
{
	float ground_speed_body;
	float _R_nb00, _R_nb10;
	Vector2f ground_speed_vector;
	Vector2f yaw_vector;

	if (_global_pos_valid) {
		/* get ground speed vector */
		MATHLIB_ASSERT (Vector2f_init_components (&ground_speed_vector, _global_pos.vx, _global_pos.vy));

		/* rotate with current attitude */
		MATHLIB_ASSERT (Dcm_get_data(&_R_nb, &_R_nb00, 0, 0));
		MATHLIB_ASSERT (Dcm_get_data(&_R_nb, &_R_nb10, 1, 0));
		MATHLIB_ASSERT (Vector2f_init_components (&yaw_vector, _R_nb00, _R_nb10));

		MATHLIB_ASSERT (Vector2f_normalize (&yaw_vector));
		MATHLIB_ASSERT (Vector2f_mul_Vector2f (&yaw_vector, &ground_speed_body, &ground_speed_vector));

		/*
		 * Ground speed undershoot is the amount of ground velocity not reached
		 * by the plane. Consequently it is zero if airspeed is >= min ground speed
		 * and positive if airspeed < min ground speed.
		 *
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away.
		 */
		_groundspeed_undershoot = max (_fw_position_control_parameters.airspeed_min - ground_speed_body, 0.0f);

	} else {
		_groundspeed_undershoot = 0;
	}

	return 0;
}

int control_position(Vector *current_position, Vector *ground_speed)
{
	bool_t setpoint = 1 /* true */;
	bool_t was_circle_mode, climb_out;
	Vector2f next_wp, prev_wp, rtl_pos, loiter_hold_pos;

	float prev_wp0, prev_wp1, current_position0, current_position1;
	float wp_distance;
	float flare_angle_rad, throttle_land, airspeed_land, airspeed_approach;
	//float land_pitch_min;

	calculate_gndspeed_undershoot();

	float eas2tas = 1.0f; // XXX calculate actual number based on current measurements
	float baro_altitude = _global_pos.altitude; // XXX re-visit

	/* filter speed and altitude for controller */
	Vector3f accel_body;
	Vector3f accel_earth;
	Dcm _R_nb_transposed;
	MATHLIB_ASSERT (Vector3f_init_components (&accel_body, _accel.x, _accel.y, _accel.z));
	MATHLIB_ASSERT (Dcm_transpose (&_R_nb, &_R_nb_transposed));
	MATHLIB_ASSERT (Dcm_mul_Vector (&_R_nb_transposed, &accel_earth, &accel_body));


	ECL_tecs_update_50hz(baro_altitude, _airspeed.indicated_airspeed_m_s, &_R_nb, &accel_body, &accel_earth);
	float altitude_error = _global_triplet.current.altitude - _global_pos.altitude;

	/* no throttle limit as default */
	float throttle_max = 1.0f;


	/* AUTONOMOUS FLIGHT */
	// XXX this should only execute if auto AND safety off (actuators active),
	// else integrators should be constantly reset.
	if (_control_flags.flag_control_position_enabled) {

		/* get circle mode */
		was_circle_mode = ECL_l1_position_controller_circle_mode();

		/* restore speed weight, in case changed intermittently (e.g. in landing handling) */
		ECL_tecs_set_speed_weight(_fw_position_control_parameters.speed_weight);

		/* execute navigation once we have a setpoint */
		if (_setpoint_valid) {

			/* current waypoint (the one currently heading for) */
			MATHLIB_ASSERT (Vector2f_init_components (&next_wp, _global_triplet.current.latitude, _global_triplet.current.longitude));

			/* previous waypoint */
			if (_global_triplet.previous_valid) {
				MATHLIB_ASSERT (Vector2f_init_components (&prev_wp, _global_triplet.previous.latitude, _global_triplet.previous.longitude));
			} else {
				/*
				 * No valid previous waypoint, go for the current wp.
				 * This is automatically handled by the L1 library.
				 */
				MATHLIB_ASSERT (Vector2f_init_components (&prev_wp, _global_triplet.current.latitude, _global_triplet.current.longitude));
			}

			// XXX add RTL switch
			if (_global_triplet.current.nav_cmd == navigation_command_rtl && _launch_valid) {
				MATHLIB_ASSERT (Vector2f_init_components (&rtl_pos, _launch_lat, _launch_lon));

				ECL_l1_position_controller_navigate_waypoints(&rtl_pos, &rtl_pos, current_position, ground_speed);
				_att_sp.roll_body = ECL_l1_position_controller_nav_roll();
				_att_sp.yaw_body = ECL_l1_position_controller_nav_bearing();

				ECL_tecs_update_pitch_throttle (&_R_nb, _att.pitch, _global_pos.altitude, _launch_alt, calculate_target_airspeed(_fw_position_control_parameters.airspeed_trim),
								_airspeed.indicated_airspeed_m_s, eas2tas, 0 /* false */, radians(_fw_position_control_parameters.pitch_limit_min),
								_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise, radians(_fw_position_control_parameters.pitch_limit_min), radians(_fw_position_control_parameters.pitch_limit_max));

				// XXX handle case when having arrived at home (loiter)

			} else if (_global_triplet.current.nav_cmd == navigation_command_waypoint) {
				/* waypoint is a plain navigation waypoint */
				ECL_l1_position_controller_navigate_waypoints(&prev_wp, &next_wp, current_position, ground_speed);
				_att_sp.roll_body = ECL_l1_position_controller_nav_roll();
				_att_sp.yaw_body = ECL_l1_position_controller_nav_bearing();

				ECL_tecs_update_pitch_throttle (&_R_nb, _att.pitch, _global_pos.altitude, _global_triplet.current.altitude, calculate_target_airspeed(_fw_position_control_parameters.airspeed_trim),
								_airspeed.indicated_airspeed_m_s, eas2tas, 0 /* false */, radians(_fw_position_control_parameters.pitch_limit_min),
								_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise, radians(_fw_position_control_parameters.pitch_limit_min), radians(_fw_position_control_parameters.pitch_limit_max));

			} else if (_global_triplet.current.nav_cmd == navigation_command_loiter) {

				/* waypoint is a loiter waypoint */
				ECL_l1_position_controller_navigate_loiter(&next_wp, current_position, _global_triplet.current.loiter_radius, _global_triplet.current.loiter_direction, ground_speed);
				_att_sp.roll_body = ECL_l1_position_controller_nav_roll();
				_att_sp.yaw_body = ECL_l1_position_controller_nav_bearing();

				ECL_tecs_update_pitch_throttle(&_R_nb, _att.pitch, _global_pos.altitude, _global_triplet.current.altitude, calculate_target_airspeed(_fw_position_control_parameters.airspeed_trim),
								_airspeed.indicated_airspeed_m_s, eas2tas, 0 /* false */, radians(_fw_position_control_parameters.pitch_limit_min),
								_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise, radians(_fw_position_control_parameters.pitch_limit_min), radians(_fw_position_control_parameters.pitch_limit_max));

			} else if (_global_triplet.current.nav_cmd == navigation_command_land) {

				/* switch to heading hold for the last meters, continue heading hold after */
				MATHLIB_ASSERT (Vector2f_getX (&prev_wp, &prev_wp0));
				MATHLIB_ASSERT (Vector2f_getY (&prev_wp, &prev_wp1));
				MATHLIB_ASSERT (Vector2f_getX (current_position, &current_position0));
				MATHLIB_ASSERT (Vector2f_getY (current_position, &current_position1));
				wp_distance = get_distance_to_next_waypoint(prev_wp0, prev_wp1, current_position0, current_position1);

				//warnx("wp dist: %d, alt err: %d, noret: %s", (int)wp_distance, (int)altitude_error, (land_noreturn) ? "YES" : "NO");

				if (wp_distance < 15.0f || land_noreturn) {

					/* heading hold, along the line connecting this and the last waypoint */
					

					// if (_global_triplet.previous_valid) {
					// 	target_bearing = get_bearing_to_next_waypoint(prev_wp.getX(), prev_wp.getY(), next_wp.getX(), next_wp.getY());
					// } else {

					if (!land_noreturn)
						target_bearing = _att.yaw;
					//}

					//warnx("NORET: %d, target_bearing: %d, yaw: %d", (int)land_noreturn, (int)degrees(target_bearing), (int)degrees(_att.yaw));

					ECL_l1_position_controller_navigate_heading(target_bearing, _att.yaw, ground_speed);

					if (altitude_error > -5.0f)
						land_noreturn = 1 /* true */;

				} else {

					/* normal navigation */
					ECL_l1_position_controller_navigate_waypoints(&prev_wp, &next_wp, current_position, ground_speed);
				}

				/* do not go down too early */
				if (wp_distance > 50.0f) {
					altitude_error = (_global_triplet.current.altitude + 25.0f) - _global_pos.altitude;
				}


				_att_sp.roll_body = ECL_l1_position_controller_nav_roll();
				_att_sp.yaw_body = ECL_l1_position_controller_nav_bearing();

				/* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */
				// XXX this could make a great param

				flare_angle_rad = radians(10.0f); //radians(_global_triplet.current.param1)
				//land_pitch_min = radians(5.0f);
				throttle_land = _fw_position_control_parameters.throttle_min + (_fw_position_control_parameters.throttle_max - _fw_position_control_parameters.throttle_min) * 0.1f;
				airspeed_land = _fw_position_control_parameters.airspeed_min;
				airspeed_approach = (_fw_position_control_parameters.airspeed_min + _fw_position_control_parameters.airspeed_trim) / 2.0f;

				if (altitude_error > -4.0f) {

					/* land with minimal speed */

					/* force TECS to only control speed with pitch, altitude is only implicitely controlled now */
					ECL_tecs_set_speed_weight(2.0f);

					ECL_tecs_update_pitch_throttle (&_R_nb, _att.pitch, _global_pos.altitude, _global_triplet.current.altitude, calculate_target_airspeed(airspeed_land),
									_airspeed.indicated_airspeed_m_s, eas2tas, 0 /* false */, flare_angle_rad,
									0.0f, _fw_position_control_parameters.throttle_max, throttle_land, radians(-10.0f), radians(15.0f));

					/* kill the throttle if param requests it */
					throttle_max = min (throttle_max, _fw_position_control_parameters.throttle_land_max);

					/* limit roll motion to prevent wings from touching the ground first */
					_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-10.0f), radians(10.0f));

				} else if (wp_distance < 60.0f && altitude_error > -20.0f) {

					/* minimize speed to approach speed */

					ECL_tecs_update_pitch_throttle (&_R_nb, _att.pitch, _global_pos.altitude, _global_triplet.current.altitude, calculate_target_airspeed(airspeed_approach),
									_airspeed.indicated_airspeed_m_s, eas2tas, 0 /* false */, flare_angle_rad,
									_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise, radians(_fw_position_control_parameters.pitch_limit_min), radians(_fw_position_control_parameters.pitch_limit_max));

				} else {

					/* normal cruise speed */

					ECL_tecs_update_pitch_throttle (&_R_nb, _att.pitch, _global_pos.altitude, _global_triplet.current.altitude, calculate_target_airspeed(_fw_position_control_parameters.airspeed_trim),
									_airspeed.indicated_airspeed_m_s, eas2tas, 0 /* false */, radians(_fw_position_control_parameters.pitch_limit_min),
									_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise, radians(_fw_position_control_parameters.pitch_limit_min), radians(_fw_position_control_parameters.pitch_limit_max));
				}

			} else if (_global_triplet.current.nav_cmd == navigation_command_takeoff) {

				ECL_l1_position_controller_navigate_waypoints(&prev_wp, &next_wp, current_position, ground_speed);
				_att_sp.roll_body = ECL_l1_position_controller_nav_roll();
				_att_sp.yaw_body = ECL_l1_position_controller_nav_bearing();

				/* apply minimum pitch and limit roll if target altitude is not within 10 meters */
				if (altitude_error > 10.0f) {

					/* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
					ECL_tecs_update_pitch_throttle(&_R_nb, _att.pitch, _global_pos.altitude, _global_triplet.current.altitude, calculate_target_airspeed(_fw_position_control_parameters.airspeed_min),
									_airspeed.indicated_airspeed_m_s, eas2tas, 1 /* true */,
									radians(10.0f),		// XXX THIS NEED TO BE FIX IN	max (radians(_global_triplet.current.param1), radians(10.0f)),
									_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise, radians(_fw_position_control_parameters.pitch_limit_min), radians(_fw_position_control_parameters.pitch_limit_max));

					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-15.0f), radians(15.0f));

				} else {

					ECL_tecs_update_pitch_throttle(&_R_nb, _att.pitch, _global_pos.altitude, _global_triplet.current.altitude, calculate_target_airspeed(_fw_position_control_parameters.airspeed_trim),
									_airspeed.indicated_airspeed_m_s, eas2tas, 0 /* false */, radians(_fw_position_control_parameters.pitch_limit_min),
									_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise, radians(_fw_position_control_parameters.pitch_limit_min), radians(_fw_position_control_parameters.pitch_limit_max));
				}
			}

			// warnx("nav bearing: %8.4f bearing err: %8.4f target bearing: %8.4f", (double)_l1_control.nav_bearing(),
			//	 (double)_l1_control.bearing_error(), (double)_l1_control.target_bearing());
			// warnx("prev wp: %8.4f/%8.4f, next wp: %8.4f/%8.4f prev:%s", (double)prev_wp.getX(), (double)prev_wp.getY(),
			//	 (double)next_wp.getX(), (double)next_wp.getY(), (_global_triplet.previous_valid) ? "valid" : "invalid");

			// XXX at this point we always want no loiter hold if a mission is active
			_loiter_hold = 0 /* false */;

		} else if (_control_flags.flag_armed) {

			/* hold position, but only if armed, climb 20m in case this is engaged on ground level */

			// XXX rework with smarter state machine

			if (!_loiter_hold) {
				_loiter_hold_lat = _global_pos.latitude;
				_loiter_hold_lon = _global_pos.longitude;
				_loiter_hold_alt = _global_pos.altitude + 25.0f;
				_loiter_hold = 1 /* true */;
			}

			altitude_error = _loiter_hold_alt - _global_pos.altitude;

			MATHLIB_ASSERT (Vector2f_init_components (&loiter_hold_pos, _loiter_hold_lat, _loiter_hold_lon));

			/* loiter around current position */
			ECL_l1_position_controller_navigate_loiter(&loiter_hold_pos, current_position, _fw_position_control_parameters.loiter_hold_radius, 1, ground_speed);
			_att_sp.roll_body = ECL_l1_position_controller_nav_roll();
			_att_sp.yaw_body = ECL_l1_position_controller_nav_bearing();

			/* climb with full throttle if the altitude error is bigger than 5 meters */
			climb_out = (altitude_error > 3);

			float min_pitch;

			if (climb_out) {
				min_pitch = radians(20.0f);

			} else {
				min_pitch = radians(_fw_position_control_parameters.pitch_limit_min);
			}

			ECL_tecs_update_pitch_throttle(&_R_nb, _att.pitch, _global_pos.altitude, _loiter_hold_alt, calculate_target_airspeed(_fw_position_control_parameters.airspeed_trim),
							_airspeed.indicated_airspeed_m_s, eas2tas, climb_out, min_pitch,
							_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise,
							radians(_fw_position_control_parameters.pitch_limit_min), radians(_fw_position_control_parameters.pitch_limit_max));

			if (climb_out) {
				/* limit roll motion to ensure enough lift */
				_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-15.0f), radians(15.0f));
			}
		}

		/* reset land state */
		if (_global_triplet.current.nav_cmd != navigation_command_land) {
			land_noreturn = 0 /* false */;
		}

		if (was_circle_mode && !ECL_l1_position_controller_circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = 1 /* true */;
		}

	} else if (0/* easy mode enabled */) {

		/** EASY FLIGHT **/

		if (0/* switched from another mode to easy */) {
			_seatbelt_hold_heading = _att.yaw;
		}

		if (0/* easy on and manual control yaw non-zero */) {
			_seatbelt_hold_heading = _att.yaw + _manual.yaw;
		}

		/* climb out control */
		climb_out = 0 /* false */;

		/* user wants to climb out */
		if (_manual.pitch > 0.3f && _manual.throttle > 0.8f) {
			climb_out = 1 /* true */;
		}

		/* if in seatbelt mode, set airspeed based on manual control */

		// XXX check if ground speed undershoot should be applied here
		float seatbelt_airspeed = _fw_position_control_parameters.airspeed_min +
					 (_fw_position_control_parameters.airspeed_max - _fw_position_control_parameters.airspeed_min) *
					 _manual.throttle;

		ECL_l1_position_controller_navigate_heading(_seatbelt_hold_heading, _att.yaw, ground_speed);
		_att_sp.roll_body = ECL_l1_position_controller_nav_roll();
		_att_sp.yaw_body = ECL_l1_position_controller_nav_bearing();
		ECL_tecs_update_pitch_throttle(&_R_nb, _att.pitch, _global_pos.altitude, _global_pos.altitude + _manual.pitch * 2.0f,
						seatbelt_airspeed, _airspeed.indicated_airspeed_m_s, eas2tas,
						0 /* false */, _fw_position_control_parameters.pitch_limit_min,
						_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise,
						_fw_position_control_parameters.pitch_limit_min, _fw_position_control_parameters.pitch_limit_max);

	} else if (0/* seatbelt mode enabled */) {

		/** SEATBELT FLIGHT **/

		if (0/* switched from another mode to seatbelt */) {
			_seatbelt_hold_heading = _att.yaw;
		}

		if (0/* seatbelt on and manual control yaw non-zero */) {
			_seatbelt_hold_heading = _att.yaw + _manual.yaw;
		}

		/* if in seatbelt mode, set airspeed based on manual control */

		// XXX check if ground speed undershoot should be applied here
		float seatbelt_airspeed = _fw_position_control_parameters.airspeed_min +
					 (_fw_position_control_parameters.airspeed_max - _fw_position_control_parameters.airspeed_min) *
					 _manual.throttle;

		/* user switched off throttle */
		if (_manual.throttle < 0.1f) {
			throttle_max = 0.0f;
			/* switch to pure pitch based altitude control, give up speed */
			ECL_tecs_set_speed_weight(0.0f);
		}

		/* climb out control */
		climb_out = 0 /* false */;

		/* user wants to climb out */
		if (_manual.pitch > 0.3f && _manual.throttle > 0.8f) {
			climb_out = 1 /* true */;
		}

		ECL_l1_position_controller_navigate_heading(_seatbelt_hold_heading, _att.yaw, ground_speed);
		_att_sp.roll_body =	_manual.roll;
		_att_sp.yaw_body =	_manual.yaw;
		ECL_tecs_update_pitch_throttle(&_R_nb, _att.pitch, _global_pos.altitude, _global_pos.altitude + _manual.pitch * 2.0f,
						seatbelt_airspeed, _airspeed.indicated_airspeed_m_s, eas2tas,
						climb_out, _fw_position_control_parameters.pitch_limit_min,
						_fw_position_control_parameters.throttle_min, _fw_position_control_parameters.throttle_max, _fw_position_control_parameters.throttle_cruise,
						_fw_position_control_parameters.pitch_limit_min, _fw_position_control_parameters.pitch_limit_max);

	} else {

		/** MANUAL FLIGHT **/

		/* no flight mode applies, do not publish an attitude setpoint */
		setpoint = 0 /* false */;
	}

	_att_sp.pitch_body = ECL_tecs_get_pitch_demand();
	_att_sp.thrust = min (ECL_tecs_get_throttle_demand(), throttle_max);

	return setpoint;
}


void* fw_position_control_thread_main (void* args)
{
	/* welcome user */
	fprintf (stdout, "Position controller started\n");
	fflush(stdout);

	Vector2f ground_speed;
	Vector2f current_position;

	float deltaT, turn_distance;
	int updated;
	absolute_time usec_max_poll_wait_time = 200000;

	/* declare and safely initialize all structs */
	struct parameter_update_s p_update;
	memset(&p_update, 0, sizeof(p_update));
	memset(&_att, 0, sizeof(_att));						/**< vehicle attitude */
	memset(&_manual, 0, sizeof(_manual));			/**< r/c channel data */
	memset(&_airspeed, 0, sizeof(_airspeed));						/**< airspeed */
	memset(&_control_flags, 0, sizeof(_control_flags));		/**< vehicle status */
	memset(&_global_pos, 0, sizeof(_global_pos));		/**< global vehicle position */
	memset(&_global_triplet, 0, sizeof(_global_triplet));	/**< triplet of global setpoints */
	memset(&_accel, 0, sizeof(_accel));							/**< body frame accelerations */

	/* output structs */
	memset(&_att_sp, 0, sizeof(_att_sp));			/**< vehicle attitude setpoint */
	memset(&_nav_capabilities, 0, sizeof(_nav_capabilities));	/**< navigation capabilities */
	_nav_capabilities.turn_distance = 0.0f;


	/* abort on a nonzero return value from the parameter init */
	if (fw_position_control_param_init() != 0) {
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Position controller aborting on startup due to an error\n");
		exit(-1);
	}

	ECL_l1_position_controller_init ();
	ECL_tecs_init ();
	Dcm_init_default (&_R_nb);

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	if (_global_pos_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to vehicle_global_position topic\n");
		exit(-1);
	}

	_global_set_triplet_sub = orb_subscribe(ORB_ID(vehicle_global_position_set_triplet));
	if (_global_set_triplet_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to vehicle_global_position_set_triplet topic\n");
		exit(-1);
	}

	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	if (_att_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to vehicle_attitude topic\n");
		exit(-1);
	}

	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	if (_accel_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to sensor_accel topic\n");
		exit(-1);
	}

	_control_flags_sub = orb_subscribe(ORB_ID(vehicle_control_flags));
	if (_control_flags_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to vehicle_control_flags topic\n");
		exit(-1);
	}

	_airspeed_sub = orb_subscribe(ORB_ID(sensor_airspeed));
	if (_airspeed_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to airspeed topic\n");
		exit(-1);
	}

	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	if (_params_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to parameter_update topic\n");
		exit(-1);
	}

	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	if (_manual_control_sub < 0)
	{
		fprintf (stderr, "Position controller thread failed to subscribe to manual_control_setpoint topic\n");
		exit(-1);
	}

	/*
	 * do advertises
	 */
	_attitude_sp_adv = orb_advertise(ORB_ID(vehicle_attitude_setpoint));
	if (_attitude_sp_adv < 0) {
		fprintf (stderr, "Position controller thread failed to advertise vehicle_attitude_setpoint topic\n");
		exit(-1);
	}

	_nav_capabilities_adv = orb_advertise(ORB_ID(navigation_capabilities));
	if (_nav_capabilities_adv < 0) {
		fprintf (stderr, "Position controller thread failed to advertise navigation_capabilities topic\n");
		exit(-1);
	}


	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(ORB_ID(vehicle_control_flags), _control_flags_sub, 200000 /* usec */);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(ORB_ID(vehicle_global_position), _global_pos_sub, 50000 /* usec */);


	while (!_shutdown_all_systems) {

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

		updated = orb_check(ORB_ID(parameter_update), _params_sub);
		if (updated) {
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(parameter_update), _params_sub, &p_update);

			/* update parameters from storage */
			fw_position_control_parameters_update();
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


		/* only run controller if position changed */
		deltaT = (get_absolute_time() - last_run) / 1000000.0f;
		last_run = get_absolute_time();

		/* guard against too large deltaT's */
		if (deltaT > 1.0f)
			deltaT = 0.01f;

		/* load local copies */
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

		// XXX add timestamp check
		_global_pos_valid = 1 /* true */;

		vehicle_attitude_poll();
		vehicle_setpoint_poll();
		vehicle_accel_poll();
		vehicle_airspeed_poll();
		// vehicle_baro_poll();


		Vector2f_init_components (&ground_speed, _global_pos.vx, _global_pos.vy);
		Vector2f_init_components (&current_position, _global_pos.latitude, _global_pos.longitude);

		/*
		 * Attempt to control position, on success (= sensors present and not in manual mode),
		 * publish setpoint.
		 */
		if (control_position(&current_position, &ground_speed)) {
			/* publish the attitude setpoint */
			orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_adv, &_att_sp);

			turn_distance = ECL_l1_position_controller_switch_distance(_global_triplet.current.turn_distance_xy);

			/* lazily publish navigation capabilities */
			if (turn_distance != _nav_capabilities.turn_distance && turn_distance > 0) {

				/* set new turn distance */
				_nav_capabilities.turn_distance = turn_distance;
				orb_publish(ORB_ID(navigation_capabilities), _nav_capabilities_adv, &_nav_capabilities);
			}
		}
	}


	/*
	 * do unsubscriptions
	 */
	orb_unsubscribe(ORB_ID(vehicle_global_position), _global_pos_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_global_position_set_triplet), _global_set_triplet_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude), _att_sub, pthread_self());
	orb_unsubscribe(ORB_ID(sensor_airspeed), _airspeed_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_control_flags), _control_flags_sub, pthread_self());
	orb_unsubscribe(ORB_ID(parameter_update), _params_sub, pthread_self());
	orb_unsubscribe(ORB_ID(manual_control_setpoint), _manual_control_sub, pthread_self());
	orb_unsubscribe(ORB_ID(sensor_accel), _accel_sub, pthread_self());


	/*
	 * do unadvertises
	 */
	orb_unadvertise(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_adv, pthread_self());
	orb_unadvertise(ORB_ID(navigation_capabilities), _nav_capabilities_adv, pthread_self());

	return 0;
}
