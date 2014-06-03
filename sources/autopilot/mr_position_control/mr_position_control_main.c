/**
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include "../../uav_library/common.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/math/limits.h"
#include "../../uav_library/math/EulerAngles.h"
#include "../../uav_library/math/Vector2f.h"
#include "../../uav_library/math/Vector3f.h"
#include "../../uav_library/math/Matrix.h"
#include "../../uav_library/math/Dcm.h"

#include "mr_position_control_params.h"

#include "../../ORB/ORB.h"
#include "../../ORB/topics/parameter_update.h"
#include "../../ORB/topics/setpoint/vehicle_global_velocity_setpoint.h"
#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/vehicle_control_flags.h"
#include "../../ORB/topics/actuator/actuator_armed.h"
#include "../../ORB/topics/position/vehicle_global_position.h"
#include "../../ORB/topics/setpoint/vehicle_global_position_set_triplet.h"



#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f


static int _att_sub;		/**< vehicle attitude subscription */
static int _att_sp_sub;		/**< vehicle attitude setpoint */
static int _control_mode_sub;		/**< vehicle control mode subscription */
static int _params_sub;		/**< notification of parameter updates */
static int _manual_sub;		/**< notification of manual control updates */
static int _arming_sub;		/**< arming status of outputs */
static int _global_pos_sub;		/**< vehicle local position */
static int _pos_sp_triplet_sub;		/**< position setpoint triplet */

static orb_advert_t _att_sp_pub;		/**< attitude setpoint publication */
static orb_advert_t _pos_sp_triplet_pub;		/**< position setpoint triplet publication */
static orb_advert_t _global_vel_sp_pub;		/**< vehicle global velocity setpoint */

static struct vehicle_attitude_s _att;		/**< vehicle attitude */
static struct vehicle_attitude_setpoint_s _att_sp;		/**< vehicle attitude setpoint */
static struct manual_control_setpoint_s _manual;		/**< r/c channel data */
static struct vehicle_control_flags_s _control_flags;		/**< vehicle control mode */
static struct actuator_armed_s _arming;		/**< actuator arming status */
static struct vehicle_global_position_s _global_pos;		/**< vehicle global position */
static struct vehicle_global_position_set_triplet_s _pos_sp_triplet;		/**< vehicle global position setpoint triplet */
static struct vehicle_global_velocity_setpoint_s _global_vel_sp;		/**< vehicle global velocity setpoint */

static double _lat_sp;
static double _lon_sp;
static float _alt_sp;

static bool_t _reset_lat_lon_sp;
static bool_t _reset_alt_sp;
static bool_t _use_global_alt;		/**< switch between global (AMSL) and barometric altitudes */

static Vector3f _vel;
static Vector3f _vel_sp;
static Vector3f _vel_prev;		/**< velocity on previous step */



int mr_position_control_init()
{
	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_flags));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(vehicle_global_position_set_triplet));
	if (_att_sub == -1 || _att_sp_sub == -1 || _control_mode_sub == -1 ||
		_params_sub == -1 || _manual_sub == -1 || _arming_sub == -1 ||
		_global_pos_sub == -1 || _pos_sp_triplet_sub == -1)
		return -1;

	/*
	 * publications
	 */
	_pos_sp_triplet_pub = orb_advertise(ORB_ID(vehicle_global_position_set_triplet));
	_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint));
	_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint));
	if (_att_sp_pub == -1 || _pos_sp_triplet_pub == -1 || _global_vel_sp_pub == -1)
		return -1;

	_lat_sp = 0.0;
	_lon_sp = 0.0;
	_alt_sp = 0.0f;

	_reset_lat_lon_sp = 1;
	_reset_alt_sp = 1;
	_use_global_alt = 1;

	memset(&_att, 0, sizeof(_att));
	memset(&_att_sp, 0, sizeof(_att_sp));
	memset(&_manual, 0, sizeof(_manual));
	memset(&_control_flags, 0, sizeof(_control_flags));
	memset(&_arming, 0, sizeof(_arming));
	memset(&_global_pos, 0, sizeof(_global_pos));
	memset(&_pos_sp_triplet, 0, sizeof(_pos_sp_triplet));
	memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));

	MATHLIB_ASSERT (Vector3f_init_zero (&mr_position_control_parameters.pos_p));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_position_control_parameters.vel_p));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_position_control_parameters.vel_i));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_position_control_parameters.vel_d));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_position_control_parameters.vel_max));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_position_control_parameters.vel_ff));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_position_control_parameters.sp_offs_max));

	MATHLIB_ASSERT (Vector3f_init_zero (&_vel));
	MATHLIB_ASSERT (Vector3f_init_zero (&_vel_sp));
	MATHLIB_ASSERT (Vector3f_init_zero (&_vel_prev));

	return mr_position_control_params_init();
}


void
mr_position_control_poll_subscriptions()
{
	if (orb_check(ORB_ID(vehicle_attitude), _att_sub))
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

	if (orb_check(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub))
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);

	if (orb_check(ORB_ID(vehicle_control_flags), _control_mode_sub))
		orb_copy(ORB_ID(vehicle_control_flags), _control_mode_sub, &_control_flags);

	if (orb_check(ORB_ID(manual_control_setpoint), _manual_sub))
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);

	if (orb_check(ORB_ID(actuator_armed), _arming_sub))
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);

	if (orb_check(ORB_ID(vehicle_global_position), _global_pos_sub))
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
}

float
mr_position_control_scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}


void
mr_position_control_reset_lat_lon_sp()
{
	if (_reset_lat_lon_sp) {
		_reset_lat_lon_sp = 0;
		_lat_sp = _global_pos.latitude;
		_lon_sp = _global_pos.longitude;
		//mavlink_log_info(_mavlink_fd, "[mpc] reset lat/lon sp: %.7f, %.7f", _lat_sp, _lon_sp);
	}
}

void
mr_position_control_reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = 0;
		_alt_sp = _use_global_alt ? _global_pos.altitude : _global_pos.baro_alt;
		//mavlink_log_info(_mavlink_fd, "[mpc] reset alt (%s) sp: %.2f", _use_global_alt ? "AMSL" : "baro", (double)_alt_sp);
	}
}

void
mr_position_control_select_alt(bool_t global)
{
	if (global != _use_global_alt) {
		_use_global_alt = global;

		if (global) {
			/* switch from barometric to global altitude */
			_alt_sp += _global_pos.altitude - _global_pos.baro_alt;

		} else {
			/* switch from global to barometric altitude */
			_alt_sp += _global_pos.baro_alt - _global_pos.altitude;
		}
	}
}


int mr_position_control_thread_main (void* args)
{
	/* welcome user */
	fprintf (stdout, "Multirotor position controller started\n");
	fflush(stdout);

	/* abort on a nonzero return value from the parameter init */
	if (mr_position_control_init() != 0) {
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Position estimator aborting on startup due to an error\n");
		exit(-1);
	}

	struct parameter_update_s param_update;
	bool_t updated;

	/* initialize values of critical structs until first regular update */
	_arming.armed = 0;

	/* get an initial update for all sensor and status data */
	mr_position_control_poll_subscriptions();

	bool_t reset_int_z = 1;
	bool_t reset_int_z_manual = 0;
	bool_t reset_int_xy = 1;
	bool_t was_armed = 0;

	absolute_time t, t_prev = 0;

	const float alt_ctl_dz = 0.2f;
	const float pos_ctl_dz = 0.05f;
	float dt;
	int i;

	Vector3f v3f_temp;
	Vector2f v2f_temp;
	Vector3f sp_move_rate;
	Vector3f thrust_int;
	Vector3f pos_sp_offs;
	MATHLIB_ASSERT (Vector3f_init_zero (&v3f_temp));
	MATHLIB_ASSERT (Vector3f_init_zero (&sp_move_rate));
	MATHLIB_ASSERT (Vector3f_init_zero (&thrust_int));
	MATHLIB_ASSERT (Vector3f_init_zero (&pos_sp_offs));

	Matrix R;
	MATHLIB_ASSERT (Matrix_init_identity(&R, 3));

	while (!_shutdown_all_systems) {
		updated = orb_poll(ORB_ID(vehicle_global_position), _global_pos_sub, 5000);
		if (!updated)
			continue;
		else if (updated < 0)
		{
			/* this is undesirable but not much we can do */
			fprintf (stderr, "Position controller failed to poll global position\n");
			continue;
		}

		updated = orb_check (ORB_ID(parameter_update), _params_sub);
		if (updated) {
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

			/* update parameters */
			mr_position_control_params_update();
		}

		mr_position_control_poll_subscriptions();
		t = get_absolute_time();
		dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		if (_control_flags.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_lat_lon_sp = 1;
			_reset_alt_sp = 1;
			reset_int_z = 1;
			reset_int_xy = 1;
		}

		was_armed = _control_flags.flag_armed;

		if (_control_flags.flag_control_altitude_enabled ||
		    _control_flags.flag_control_position_enabled ||
		    _control_flags.flag_control_climb_rate_enabled ||
		    _control_flags.flag_control_velocity_enabled) {

			_vel.data[0] = _global_pos.vx;
			_vel.data[1] = _global_pos.vy;
			_vel.data[2] = _global_pos.vz;

			MATHLIB_ASSERT (Vector3f_init_zero (&sp_move_rate));

			float alt = _global_pos.altitude;

			/* select control source */
			if (_control_flags.flag_control_manual_enabled) {
				/* select altitude source and update setpoint */
				mr_position_control_select_alt(_global_pos.valid);

				if (!_use_global_alt) {
					alt = _global_pos.baro_alt;
				}

				/* manual control */
				if (_control_flags.flag_control_altitude_enabled) {
					/* reset alt setpoint to current altitude if needed */
					mr_position_control_reset_alt_sp();

					/* move altitude setpoint with throttle stick */
					sp_move_rate.data[2] = -mr_position_control_scale_control(_manual.thrust - 0.5f, 0.5f, alt_ctl_dz);
				}

				if (_control_flags.flag_control_position_enabled) {
					/* reset lat/lon setpoint to current position if needed */
					mr_position_control_reset_lat_lon_sp();

					/* move position setpoint with roll/pitch stick */
					sp_move_rate.data[0] = mr_position_control_scale_control(-_manual.pitch / mr_position_control_parameters.rc_scale_pitch, 1.0f, pos_ctl_dz);
					sp_move_rate.data[1] = mr_position_control_scale_control(_manual.roll / mr_position_control_parameters.rc_scale_roll, 1.0f, pos_ctl_dz);
				}

				/* limit setpoint move rate */
				float sp_move_norm;
				MATHLIB_ASSERT (Vector3f_length (&sp_move_rate, &sp_move_norm));

				if (sp_move_norm > 1.0f) {
					MATHLIB_ASSERT (Vector3f_div_float (&sp_move_rate, sp_move_norm));
				}

				/* scale to max speed and rotate around yaw */
				Dcm R_yaw_sp;
				EulerAngles e_temp;
				MATHLIB_ASSERT (EulerAngles_init_components (&e_temp, 0.0f, 0.0f, _att_sp.yaw_body));
				MATHLIB_ASSERT (Dcm_init_EulerAngles (&R_yaw_sp, &e_temp));
				MATHLIB_ASSERT (Vector3f_emul_Vector3f (&sp_move_rate, &v3f_temp, &mr_position_control_parameters.vel_max));
				MATHLIB_ASSERT (Dcm_mul_Vector (&R_yaw_sp, &sp_move_rate, &v3f_temp));

				/* move position setpoint */
				add_vector_to_global_position(_lat_sp, _lon_sp, sp_move_rate.data[0] * dt, sp_move_rate.data[1] * dt, &_lat_sp, &_lon_sp);
				_alt_sp -= sp_move_rate.data[2] * dt;

				/* check if position setpoint is too far from actual position */
				MATHLIB_ASSERT (Vector3f_init_zero (&pos_sp_offs));

				if (_control_flags.flag_control_position_enabled) {
					get_vector_to_next_waypoint_fast(_global_pos.latitude, _global_pos.longitude, _lat_sp, _lon_sp, &pos_sp_offs.data[0], &pos_sp_offs.data[1]);
					pos_sp_offs.data[0] /= mr_position_control_parameters.sp_offs_max.data[0];
					pos_sp_offs.data[1] /= mr_position_control_parameters.sp_offs_max.data[1];
				}

				if (_control_flags.flag_control_altitude_enabled) {
					pos_sp_offs.data[2] = -(_alt_sp - alt) / mr_position_control_parameters.sp_offs_max.data[2];
				}

				float pos_sp_offs_norm;
				MATHLIB_ASSERT (Vector3f_length (&pos_sp_offs, &pos_sp_offs_norm));

				if (pos_sp_offs_norm > 1.0f) {
					MATHLIB_ASSERT (Vector3f_div_float (&pos_sp_offs, pos_sp_offs_norm));

					add_vector_to_global_position(_global_pos.latitude, _global_pos.longitude, pos_sp_offs.data[0] * mr_position_control_parameters.sp_offs_max.data[0], pos_sp_offs.data[1] * mr_position_control_parameters.sp_offs_max.data[1], &_lat_sp, &_lon_sp);
					_alt_sp = alt - pos_sp_offs.data[2] * mr_position_control_parameters.sp_offs_max.data[2];
				}

				/* fill position setpoint triplet */
				_pos_sp_triplet.previous_valid = 1;
				_pos_sp_triplet.current_valid = 1;
				_pos_sp_triplet.next_valid = 1;

				_pos_sp_triplet.current.nav_cmd = navigation_command_waypoint;
				_pos_sp_triplet.current.latitude = _lat_sp;
				_pos_sp_triplet.current.longitude = _lon_sp;
				_pos_sp_triplet.current.altitude = _alt_sp;
				_pos_sp_triplet.current.yaw = _att_sp.yaw_body;
				_pos_sp_triplet.current.loiter_radius = 0.0f;
				_pos_sp_triplet.current.loiter_direction = 1.0f;

				/* publish position setpoint triplet */
				orb_publish(ORB_ID(vehicle_global_position_set_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);

			} else {
				/* always use AMSL altitude for AUTO */
				mr_position_control_select_alt(1);

				/* AUTO */
				if (orb_check(ORB_ID(vehicle_global_position_set_triplet), _pos_sp_triplet_sub))
					orb_copy(ORB_ID(vehicle_global_position_set_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

				if (_pos_sp_triplet.current_valid) {
					/* in case of interrupted mission don't go to waypoint but stay at current position */
					_reset_lat_lon_sp = 1;
					_reset_alt_sp = 1;

					/* update position setpoint */
					_lat_sp = _pos_sp_triplet.current.latitude;
					_lon_sp = _pos_sp_triplet.current.longitude;
					_alt_sp = _pos_sp_triplet.current.altitude;

					/* update yaw setpoint if needed */
					if (check_finite(_pos_sp_triplet.current.yaw)) {
						_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
					}

				} else {
					/* no waypoint, loiter, reset position setpoint if needed */
					mr_position_control_reset_lat_lon_sp();
					mr_position_control_reset_alt_sp();
				}
			}

			if (!_control_flags.flag_control_manual_enabled && _pos_sp_triplet.current_valid && _pos_sp_triplet.current.nav_cmd == navigation_command_none) {
				/* idle state, don't run controller and set zero thrust */
				MATHLIB_ASSERT (Matrix_init_identity (&R, 3));
				memcpy(&_att_sp.R_body, &R.data[0][0], sizeof(_att_sp.R_body));
				_att_sp.R_valid = 1;

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _att.yaw;
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = get_absolute_time();

				/* publish attitude setpoint */
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

			} else {
				/* run position & altitude controllers, calculate velocity setpoint */
				Vector3f pos_err;
				MATHLIB_ASSERT (Vector3f_init_zero (&pos_err));
				get_vector_to_next_waypoint_fast(_global_pos.latitude, _global_pos.longitude, _lat_sp, _lon_sp, &pos_err.data[0], &pos_err.data[1]);
				pos_err.data[2] = -(_alt_sp - alt);

				// _vel_sp = pos_err.emult(mr_position_control_parameters.pos_p) + sp_move_rate.emult(mr_position_control_parameters.vel_ff);
				MATHLIB_ASSERT (Vector3f_emul_Vector3f (&pos_err, &_vel_sp, &mr_position_control_parameters.pos_p));
				MATHLIB_ASSERT (Vector3f_emul_Vector3f (&sp_move_rate, &v3f_temp, &mr_position_control_parameters.vel_ff));
				MATHLIB_ASSERT (Vector3f_add_Vector3f (&_vel_sp, &v3f_temp));

				if (!_control_flags.flag_control_altitude_enabled) {
					_reset_alt_sp = 1;
					_vel_sp.data[2] = 0.0f;
				}

				if (!_control_flags.flag_control_position_enabled) {
					_reset_lat_lon_sp = 1;
					_vel_sp.data[0] = 0.0f;
					_vel_sp.data[1] = 0.0f;
				}

				/* use constant descend rate when landing, ignore altitude setpoint */
				if (!_control_flags.flag_control_manual_enabled && _pos_sp_triplet.current_valid && _pos_sp_triplet.current.nav_cmd == navigation_command_land) {
					_vel_sp.data[2] = mr_position_control_parameters.land_speed;
				}

				if (!_control_flags.flag_control_manual_enabled) {
					/* limit 3D speed only in non-manual modes */
					float vel_sp_norm;
					MATHLIB_ASSERT (Vector3f_ediv_Vector3f (&_vel_sp, &v3f_temp, &mr_position_control_parameters.vel_max));
					MATHLIB_ASSERT (Vector3f_length (&v3f_temp, &vel_sp_norm));

					if (vel_sp_norm > 1.0f) {
						MATHLIB_ASSERT (Vector3f_div_float (&_vel_sp, vel_sp_norm));
					}
				}

				_global_vel_sp.vx = _vel_sp.data[0];
				_global_vel_sp.vy = _vel_sp.data[1];
				_global_vel_sp.vz = _vel_sp.data[2];

				/* publish velocity setpoint */
				orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

				if (_control_flags.flag_control_climb_rate_enabled || _control_flags.flag_control_velocity_enabled) {
					/* reset integrals if needed */
					if (_control_flags.flag_control_climb_rate_enabled) {
						if (reset_int_z) {
							reset_int_z = 0;
							float thr;
							thr = mr_position_control_parameters.thr_min;

							if (reset_int_z_manual) {
								thr = _manual.thrust;

								if (thr < mr_position_control_parameters.thr_min) {
									thr = mr_position_control_parameters.thr_min;

								} else if (thr > mr_position_control_parameters.thr_max) {
									thr = mr_position_control_parameters.thr_max;
								}
							}

							thrust_int.data[2] = -thr;
						}

					} else {
						reset_int_z = 1;
					}

					if (_control_flags.flag_control_velocity_enabled) {
						if (reset_int_xy) {
							reset_int_xy = 0;
							thrust_int.data[0] = 0.0f;
							thrust_int.data[1] = 0.0f;
						}

					} else {
						reset_int_xy = 1;
					}

					/* velocity error */
					Vector3f vel_err;
					MATHLIB_ASSERT (Vector3f_init_Vector3f (&vel_err, &_vel_sp));
					MATHLIB_ASSERT (Vector3f_sub_Vector3f (&vel_err, &_vel));

					/* derivative of velocity error, not includes setpoint acceleration */
					Vector3f vel_err_d; // = (sp_move_rate - _vel).emult(mr_position_control_parameters.pos_p) - (_vel - _vel_prev) / dt;
					MATHLIB_ASSERT (Vector3f_init_Vector3f (&v3f_temp, &sp_move_rate));
					MATHLIB_ASSERT (Vector3f_sub_Vector3f (&v3f_temp, &_vel));
					MATHLIB_ASSERT (Vector3f_emul_Vector3f (&v3f_temp, &vel_err_d, &mr_position_control_parameters.pos_p));
					MATHLIB_ASSERT (Vector3f_init_Vector3f (&v3f_temp, &_vel));
					MATHLIB_ASSERT (Vector3f_sub_Vector3f (&v3f_temp, &_vel_prev));
					MATHLIB_ASSERT (Vector3f_div_float (&v3f_temp, dt));
					MATHLIB_ASSERT (Vector3f_sub_Vector3f (&vel_err_d, &v3f_temp));

					MATHLIB_ASSERT (Vector3f_init_Vector3f (&_vel_prev, &_vel));

					/* thrust vector in NED frame */
					Vector3f thrust_sp; // = vel_err.emult(mr_position_control_parameters.vel_p) + vel_err_d.emult(mr_position_control_parameters.vel_d) + thrust_int;
					MATHLIB_ASSERT (Vector3f_emul_Vector3f (&vel_err, &thrust_sp, &mr_position_control_parameters.vel_p));
					MATHLIB_ASSERT (Vector3f_emul_Vector3f (&vel_err_d, &v3f_temp, &mr_position_control_parameters.vel_d));
					MATHLIB_ASSERT (Vector3f_add_Vector3f (&thrust_sp, &v3f_temp));
					MATHLIB_ASSERT (Vector3f_add_Vector3f (&thrust_sp, &thrust_int));


					if (!_control_flags.flag_control_velocity_enabled) {
						thrust_sp.data[0] = 0.0f;
						thrust_sp.data[1] = 0.0f;
					}

					if (!_control_flags.flag_control_climb_rate_enabled) {
						thrust_sp.data[2] = 0.0f;
					}

					/* limit thrust vector and check for saturation */
					bool_t saturation_xy = 0;
					bool_t saturation_z = 0;

					/* limit min lift */
					float thr_min = mr_position_control_parameters.thr_min;

					if (!_control_flags.flag_control_velocity_enabled && thr_min < 0.0f) {
						/* don't allow downside thrust direction in manual attitude mode */
						thr_min = 0.0f;
					}

					float tilt_max = mr_position_control_parameters.tilt_max;

					/* adjust limits for landing mode */
					if (!_control_flags.flag_control_manual_enabled && _pos_sp_triplet.current_valid && _pos_sp_triplet.current.nav_cmd == navigation_command_land) {
						/* limit max tilt and min lift when landing */
						tilt_max = mr_position_control_parameters.land_tilt_max;

						if (thr_min < 0.0f)
							thr_min = 0.0f;
					}

					/* limit min lift */
					if (-thrust_sp.data[2] < thr_min) {
						thrust_sp.data[2] = -thr_min;
						saturation_z = 1;
					}

					if (_control_flags.flag_control_velocity_enabled) {
						/* limit max tilt */
						if (thr_min >= 0.0f && tilt_max < M_PI / 2 - 0.05f) {
							/* absolute horizontal thrust */
							float thrust_sp_xy_len;
							MATHLIB_ASSERT (Vector2f_init_components (&v2f_temp, thrust_sp.data[0], thrust_sp.data[1]));
							MATHLIB_ASSERT (Vector2f_length (&v2f_temp, &thrust_sp_xy_len));

							if (thrust_sp_xy_len > 0.01f) {
								/* max horizontal thrust for given vertical thrust*/
								float thrust_xy_max = -thrust_sp.data[2] * tanf(tilt_max);

								if (thrust_sp_xy_len > thrust_xy_max) {
									float k = thrust_xy_max / thrust_sp_xy_len;
									thrust_sp.data[0] *= k;
									thrust_sp.data[1] *= k;
									saturation_xy = 1;
								}
							}
						}

					} else {
						/* thrust compensation for altitude only control mode */
						float att_comp;

						if (_att.R[2][2] > TILT_COS_MAX) {
							att_comp = 1.0f / _att.R[2][2];

						} else if (_att.R[2][2] > 0.0f) {
							att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _att.R[2][2] + 1.0f;
							saturation_z = 1;

						} else {
							att_comp = 1.0f;
							saturation_z = 1;
						}

						thrust_sp.data[2] *= att_comp;
					}

					/* limit max thrust */
					float thrust_abs;
					MATHLIB_ASSERT (Vector3f_length (&thrust_sp, &thrust_abs));

					if (thrust_abs > mr_position_control_parameters.thr_max) {
						if (thrust_sp.data[2] < 0.0f) {
							if (-thrust_sp.data[2] > mr_position_control_parameters.thr_max) {
								/* thrust Z component is too large, limit it */
								thrust_sp.data[0] = 0.0f;
								thrust_sp.data[1] = 0.0f;
								thrust_sp.data[2] = -mr_position_control_parameters.thr_max;
								saturation_xy = 1;
								saturation_z = 1;

							} else {
								/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
								float thrust_xy_max = sqrtf(mr_position_control_parameters.thr_max * mr_position_control_parameters.thr_max - thrust_sp.data[2] * thrust_sp.data[2]);
								float thrust_xy_abs;
								MATHLIB_ASSERT (Vector2f_init_components (&v2f_temp, thrust_sp.data[0], thrust_sp.data[1]));
								MATHLIB_ASSERT (Vector2f_length (&v2f_temp, &thrust_xy_abs));

								float k = thrust_xy_max / thrust_xy_abs;
								thrust_sp.data[0] *= k;
								thrust_sp.data[1] *= k;
								saturation_xy = 1;
							}

						} else {
							/* Z component is negative, going down, simply limit thrust vector */
							float k = mr_position_control_parameters.thr_max / thrust_abs;
							MATHLIB_ASSERT (Vector3f_mul_float (&thrust_sp, k));
							saturation_xy = 1;
							saturation_z = 1;
						}

						thrust_abs = mr_position_control_parameters.thr_max;
					}

					/* update integrals */
					if (_control_flags.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int.data[0] += vel_err.data[0] * mr_position_control_parameters.vel_i.data[0] * dt;
						thrust_int.data[1] += vel_err.data[1] * mr_position_control_parameters.vel_i.data[1] * dt;
					}

					if (_control_flags.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int.data[2] += vel_err.data[2] * mr_position_control_parameters.vel_i.data[2] * dt;

						/* protection against flipping on ground when landing */
						if (thrust_int.data[2] > 0.0f)
							thrust_int.data[2] = 0.0f;
					}

					/* calculate attitude setpoint from thrust vector */
					if (_control_flags.flag_control_velocity_enabled) {
						/* desired body_z axis = -normalize(thrust_vector) */
						Vector3f body_x;
						Vector3f body_y;
						Vector3f body_z;
						MATHLIB_ASSERT (Vector3f_init_zero (&body_x));
						MATHLIB_ASSERT (Vector3f_init_zero (&body_y));
						MATHLIB_ASSERT (Vector3f_init_zero (&body_z));

						if (thrust_abs > SIGMA) {
							MATHLIB_ASSERT (Vector3f_init_Vector3f (&body_z, &thrust_sp));
							MATHLIB_ASSERT (Vector3f_change_sign (&body_z));
							MATHLIB_ASSERT (Vector3f_div_float (&body_z, thrust_abs));

						} else {
							/* no thrust, set Z axis to safe value */
							MATHLIB_ASSERT (Vector3f_init_zero (&body_z));
							body_z.data[2] = 1.0f;
						}

						/* vector of desired yaw direction in XY plane, rotated by PI/2 */
						Vector3f y_C;
						MATHLIB_ASSERT(Vector3f_init_components(&y_C, -sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f));

						if (fabsf(body_z.data[2]) > SIGMA) {
							/* desired body_x axis, orthogonal to body_z */
							MATHLIB_ASSERT (Vector3f_init_Vector3f (&body_x, &y_C));
							MATHLIB_ASSERT (Vector3f_cross_Vector3f (&body_x, &body_z));

							/* keep nose to front while inverted upside down */
							if (body_z.data[2] < 0.0f) {
								MATHLIB_ASSERT (Vector3f_change_sign (&body_x));
							}

							MATHLIB_ASSERT (Vector3f_normalize (&body_x));

						} else {
							/* desired thrust is in XY plane, set X downside to construct correct matrix,
							 * but yaw component will not be used actually */
							MATHLIB_ASSERT (Vector3f_init_zero (&body_x));
							body_x.data[2] = 1.0f;
						}

						/* desired body_y axis */
						MATHLIB_ASSERT (Vector3f_init_Vector3f (&body_y, &body_z));
						MATHLIB_ASSERT (Vector3f_cross_Vector3f (&body_y, &body_x));

						/* fill rotation matrix */
						for (i = 0; i < 3; i++) {
							R.data[i][0] = body_x.data[i];
							R.data[i][1] = body_y.data[i];
							R.data[i][2] = body_z.data[i];
						}

						/* copy rotation matrix to attitude setpoint topic */
						memcpy(&_att_sp.R_body, &R.data[0][0], sizeof(_att_sp.R_body));
						_att_sp.R_valid = 1;

						/* calculate euler angles, for logging only, must not be used for control */
						EulerAngles euler;
						MATHLIB_ASSERT (EulerAngles_init_Dcm (&euler, &R));

						_att_sp.roll_body = euler.data[0];
						_att_sp.pitch_body = euler.data[1];
						/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */
					}

					_att_sp.thrust = thrust_abs;
					_att_sp.timestamp = get_absolute_time();

					/* publish attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

				} else {
					reset_int_z = 1;
				}
			}

		} else {
			/* position controller disabled, reset setpoints */
			_reset_alt_sp = 1;
			_reset_lat_lon_sp = 1;
			reset_int_z = 1;
			reset_int_xy = 1;
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = _control_flags.flag_armed && _control_flags.flag_control_manual_enabled && !_control_flags.flag_control_climb_rate_enabled;
	}


	/*
	 * do subscriptions
	 */
	orb_unsubscribe(ORB_ID(vehicle_attitude), _att_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_control_flags), _control_mode_sub, pthread_self());
	orb_unsubscribe(ORB_ID(parameter_update), _params_sub, pthread_self());
	orb_unsubscribe(ORB_ID(manual_control_setpoint), _manual_sub, pthread_self());
	orb_unsubscribe(ORB_ID(actuator_armed), _arming_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_global_position), _global_pos_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_global_position_set_triplet), _pos_sp_triplet_sub, pthread_self());

	/*
	 * publications
	 */
	orb_unadvertise(ORB_ID(vehicle_global_position_set_triplet), _pos_sp_triplet_pub, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, pthread_self());

	return 0;
}
