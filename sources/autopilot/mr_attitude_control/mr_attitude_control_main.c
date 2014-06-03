/**
 * @file mr_attitude_control_main.c
 * Multicopter attitude controller.
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
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
#include "../../uav_library/math/Quaternion.h"
#include "../../uav_library/math/Vector2f.h"
#include "../../uav_library/math/Vector3f.h"
#include "../../uav_library/math/Matrix.h"
#include "../../uav_library/math/Dcm.h"

#include "mr_attitude_control_params.h"

#include "../../ORB/ORB.h"
#include "../../ORB/topics/parameter_update.h"
#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/vehicle_control_flags.h"
#include "../../ORB/topics/actuator/actuator_armed.h"



#define YAW_DEADZONE 0.05f
#define MIN_TAKEOFF_THRUST 0.2f
#define RATES_I_LIMIT 0.3f


static int _v_att_sub; /**< vehicle attitude subscription */
static int _v_att_sp_sub; /**< vehicle attitude setpoint subscription */
static int _v_rates_sp_sub; /**< vehicle rates setpoint subscription */
static int _v_control_flags_sub; /**< vehicle control mode subscription */
static int _params_sub; /**< parameter updates subscription */
static int _manual_control_sp_sub; /**< manual control setpoint subscription */
static int _armed_sub; /**< arming status subscription */

static orb_advert_t _att_sp_pub; /**< attitude setpoint publication */
static orb_advert_t _v_rates_sp_pub; /**< rate setpoint publication */
static orb_advert_t _actuators_0_pub; /**< attitude actuator controls publication */

static struct vehicle_attitude_s _v_att; /**< vehicle attitude */
static struct vehicle_attitude_setpoint_s _v_att_sp; /**< vehicle attitude setpoint */
static struct vehicle_rates_setpoint_s _v_rates_sp; /**< vehicle rates setpoint */
static struct manual_control_setpoint_s _manual_control_sp; /**< manual control setpoint */
static struct vehicle_control_flags_s _v_control_flags; /**< vehicle control mode */
static struct actuator_controls_s _actuators; /**< actuator controls */
static struct actuator_armed_s _armed; /**< actuator arming status */

static float _thrust_sp; /**< thrust setpoint */
static bool_t _reset_yaw_sp; /**< reset yaw setpoint flag */

static Vector3f _rates_prev; /**< angular rates on previous step */
static Vector3f _rates_sp; /**< angular rates setpoint */
static Vector3f _rates_int; /**< angular rates integral error */
static Vector3f _att_control; /**< attitude control vector */

static Matrix _I; /**< identity matrix */

int _test_counter = 0;	// XXX to remove


int mr_attitude_control_init()
{
	/*
	 * subscriptions
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_flags_sub = orb_subscribe(ORB_ID(vehicle_control_flags));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	if (_v_att_sp_sub == -1 || _v_rates_sp_sub == -1 ||
		_v_att_sub == -1 || _v_control_flags_sub == -1 ||
		_params_sub == -1 || _manual_control_sp_sub == -1 || _armed_sub == -1)
		return -1;

	/*
	 * publications
	 */
	_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint));
	_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0));
	if (_att_sp_pub == -1 || _v_rates_sp_pub == -1 || _actuators_0_pub == -1)
		return -1;

	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_flags, 0, sizeof(_v_control_flags));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));

	MATHLIB_ASSERT (Vector3f_init_zero (&mr_attitude_control_parameters.att_p));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_attitude_control_parameters.rate_p));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_attitude_control_parameters.rate_i));
	MATHLIB_ASSERT (Vector3f_init_zero (&mr_attitude_control_parameters.rate_d));

	MATHLIB_ASSERT (Vector3f_init_zero (&_rates_prev));
	MATHLIB_ASSERT (Vector3f_init_zero (&_rates_sp));
	MATHLIB_ASSERT (Vector3f_init_zero (&_rates_int));
	MATHLIB_ASSERT (Vector3f_init_zero (&_att_control));
	MATHLIB_ASSERT (Matrix_init_identity(&_I, 3));

	_thrust_sp = 0.0f;

	return mr_attitude_control_params_init();
}



void
mr_attitude_control_vehicle_control_flags_poll()
{
	/* Check HIL state if vehicle status has changed */
	if (orb_check(ORB_ID(vehicle_control_flags), _v_control_flags_sub))
		orb_copy(ORB_ID(vehicle_control_flags), _v_control_flags_sub, &_v_control_flags);
}

void
mr_attitude_control_vehicle_manual_poll()
{
	/* get pilots inputs */
	if (orb_check(ORB_ID(manual_control_setpoint), _manual_control_sp_sub))
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
}

void
mr_attitude_control_vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	if (orb_check(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub))
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
}

void
mr_attitude_control_vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	if (orb_check(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub))
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
}

void
mr_attitude_control_arming_status_poll()
{
	/* check if there is a new setpoint */
	if (orb_check(ORB_ID(actuator_armed), _armed_sub))
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
}

/*
 * Attitude controller.
 * Input: 'manual_control_setpoint' and 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp', 'vehicle_attitude_setpoint' topic (for manual modes)
 */
int
mr_attitude_control_control_attitude(float dt)
{
	float yaw_sp_move_rate = 0.0f;
	bool_t publish_att_sp = 0;
	Vector3f v3f_temp1;
	Matrix m_temp1, m_temp2;
	float f_temp1, f_temp2;

	if (_v_control_flags.flag_control_manual_enabled) {
		/* manual input, set or modify attitude setpoint */

		if (_v_control_flags.flag_control_velocity_enabled || _v_control_flags.flag_control_climb_rate_enabled) {
			/* in assisted modes poll 'vehicle_attitude_setpoint' topic and modify it */
			mr_attitude_control_vehicle_attitude_setpoint_poll();
		}

		if (!_v_control_flags.flag_control_climb_rate_enabled) {
			/* pass throttle directly if not in altitude stabilized mode */
			_v_att_sp.thrust = _manual_control_sp.thrust;
			publish_att_sp = 1;
		}

		if (!_armed.armed) {
			/* reset yaw setpoint when disarmed */
			_reset_yaw_sp = 1;
		}

		/* move yaw setpoint in all modes */
		if (_v_att_sp.thrust < 0.1f) {
			// TODO
			//if (_status.condition_landed) {
			/* reset yaw setpoint if on ground */
			//	reset_yaw_sp = 1;
			//}
		} else {
			float yaw_dz_scaled = YAW_DEADZONE * mr_attitude_control_parameters.rc_scale_yaw;

			if (mr_attitude_control_parameters.rc_scale_yaw > 0.001f && fabs(_manual_control_sp.yaw) > yaw_dz_scaled) {
				/* move yaw setpoint */
				yaw_sp_move_rate = _manual_control_sp.yaw / mr_attitude_control_parameters.rc_scale_yaw;

				if (_manual_control_sp.yaw > 0.0f) {
					yaw_sp_move_rate -= YAW_DEADZONE;

				} else {
					yaw_sp_move_rate += YAW_DEADZONE;
				}

				yaw_sp_move_rate *= mr_attitude_control_parameters.rc_scale_yaw;
				_v_att_sp.yaw_body = _wrap_pi(_v_att_sp.yaw_body + yaw_sp_move_rate * dt);
				_v_att_sp.R_valid = 0;
				publish_att_sp = 1;
			}
		}

		/* reset yaw setpint to current position if needed */
		if (_reset_yaw_sp) {
			_reset_yaw_sp = 0;
			_v_att_sp.yaw_body = _v_att.yaw;
			_v_att_sp.R_valid = 0;
			publish_att_sp = 1;
		}

		if (!_v_control_flags.flag_control_velocity_enabled) {
			/* update attitude setpoint if not in position control mode */
			_v_att_sp.roll_body = _manual_control_sp.roll;
			_v_att_sp.pitch_body = _manual_control_sp.pitch;
			_v_att_sp.R_valid = 0;
			publish_att_sp = 1;
		}

	} else {
		/* in non-manual mode use 'vehicle_attitude_setpoint' topic */
		mr_attitude_control_vehicle_attitude_setpoint_poll();

		/* reset yaw setpoint after non-manual control mode */
		_reset_yaw_sp = 1;
	}

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	EulerAngles e_temp;
	Matrix R_sp;
	MATHLIB_ASSERT (Matrix_init_zero (&R_sp, 3, 3));

	if (_v_att_sp.R_valid) {
		/* rotation matrix in _att_sp is valid, use it */
		memcpy(&R_sp.data[0][0], &_v_att_sp.R_body, sizeof(_v_att_sp.R_body));
	} else {
		/* rotation matrix in _att_sp is not valid, use euler angles instead */
		MATHLIB_ASSERT (EulerAngles_init_components (&e_temp, _v_att_sp.roll_body, _v_att_sp.pitch_body, _v_att_sp.yaw_body));
		MATHLIB_ASSERT (Dcm_init_EulerAngles (&R_sp, &e_temp));

		/* copy rotation matrix back to setpoint struct */
		memcpy(&_v_att_sp.R_body, &R_sp.data[0][0], sizeof(_v_att_sp.R_body));
		_v_att_sp.R_valid = 1;
	}

	/* publish the attitude setpoint if needed */
	if (publish_att_sp) {
		_v_att_sp.timestamp = get_absolute_time();

		orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);
	}

	/* rotation matrix for current state */
	Matrix R, R_transposed;
	MATHLIB_ASSERT (Matrix_init_zero (&R, 3, 3));
	memcpy(&R.data[0][0], &_v_att.R[0][0], sizeof(_v_att_sp.R_body));

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	Vector3f R_z;
	Vector3f R_sp_z;
	MATHLIB_ASSERT(Vector3f_init_components(&R_z, R.data[0][2], R.data[1][2], R.data[2][2]));
	MATHLIB_ASSERT(Vector3f_init_components(&R_sp_z, R_sp.data[0][2], R_sp.data[1][2], R_sp.data[2][2]));

	/* axis and sin(angle) of desired rotation */
	Vector3f e_R;
	MATHLIB_ASSERT (Matrix_transpose (&R, &R_transposed));
	MATHLIB_ASSERT (Vector3f_init_Vector3f (&v3f_temp1, &R_z));
	MATHLIB_ASSERT (Vector3f_cross_Vector3f (&v3f_temp1, &R_sp_z));
	MATHLIB_ASSERT (Matrix_mul_Vector (&R_transposed, &e_R, &v3f_temp1));

	/* calculate angle error */
	float e_R_z_sin;
	float e_R_z_cos;
	MATHLIB_ASSERT (Vector3f_mul_Vector3f (&R_z, &e_R_z_cos, &R_sp_z));
	MATHLIB_ASSERT (Vector3f_length (&e_R, &e_R_z_sin));


	/* calculate weight for yaw control */
	float yaw_w = R_sp.data[2][2] * R_sp.data[2][2];

	/* calculate rotation matrix after roll/pitch only rotation */
	Matrix R_rp;
	float e_R_z_angle;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		Vector3f e_R_z_axis;
		MATHLIB_ASSERT (Vector3f_init_Vector3f (&e_R_z_axis, &e_R));
		MATHLIB_ASSERT (Vector3f_div_float (&e_R, e_R_z_sin));

		MATHLIB_ASSERT (Vector3f_init_Vector3f (&e_R, &e_R_z_axis));
		MATHLIB_ASSERT (Vector3f_mul_float (&e_R, e_R_z_angle));

		/* cross product matrix for e_R_axis */
		Matrix e_R_cp;
		MATHLIB_ASSERT (Matrix_init_zero (&e_R_cp, 3, 3));

		e_R_cp.data[0][1] = -e_R_z_axis.data[2];
		e_R_cp.data[0][2] = e_R_z_axis.data[1];
		e_R_cp.data[1][0] = e_R_z_axis.data[2];
		e_R_cp.data[1][2] = -e_R_z_axis.data[0];
		e_R_cp.data[2][0] = -e_R_z_axis.data[1];
		e_R_cp.data[2][1] = e_R_z_axis.data[0];

		/* rotation matrix for roll/pitch only rotation */
		//R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));
		MATHLIB_ASSERT (Matrix_init_Matrix (&m_temp1, &_I));
		MATHLIB_ASSERT (Matrix_init_Matrix (&m_temp2, &e_R_cp));
		MATHLIB_ASSERT (Matrix_mul_float (&m_temp2, e_R_z_sin));
		MATHLIB_ASSERT (Matrix_add_Matrix (&m_temp1, &m_temp2));
		MATHLIB_ASSERT (Matrix_mul_Matrix (&e_R_cp, &m_temp2, &e_R_cp));
		MATHLIB_ASSERT (Matrix_mul_float (&m_temp2, (1.0f - e_R_z_cos)));
		MATHLIB_ASSERT (Matrix_add_Matrix (&m_temp1, &m_temp2));
		MATHLIB_ASSERT (Matrix_mul_Matrix (&R, &R_rp, &m_temp1));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	Vector3f R_sp_x;
	Vector3f R_rp_x;
	MATHLIB_ASSERT(Vector3f_init_components(&R_sp_x, R_sp.data[0][0], R_sp.data[1][0], R_sp.data[2][0]));
	MATHLIB_ASSERT(Vector3f_init_components(&R_rp_x, R_rp.data[0][0], R_rp.data[1][0], R_rp.data[2][0]));

	MATHLIB_ASSERT (Vector3f_init_Vector3f (&v3f_temp1, &R_rp_x));
	MATHLIB_ASSERT (Vector3f_cross_Vector3f (&v3f_temp1, &R_sp_x));
	MATHLIB_ASSERT (Vector3f_mul_Vector3f (&v3f_temp1, &f_temp1, &R_sp_z));
	MATHLIB_ASSERT (Vector3f_mul_Vector3f (&R_rp_x, &f_temp2, &R_sp_x));
	e_R.data[2] = atan2f(f_temp1, f_temp2) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		Quaternion q;
		Vector3f e_R_d;
		float e_R_d_length;
		MATHLIB_ASSERT (Matrix_mul_Matrix (&R_transposed, &m_temp1, &R_sp));
		MATHLIB_ASSERT (Quaternion_init_Dcm (&q, &m_temp1));
		MATHLIB_ASSERT(Vector3f_init_components(&e_R_d, q.data[1], q.data[2], q.data[3]));
		MATHLIB_ASSERT(Vector3f_normalize(&e_R_d));

		MATHLIB_ASSERT (Vector3f_length (&e_R_d, &e_R_d_length));
		MATHLIB_ASSERT (Vector3f_mul_float (&e_R_d, 2.0f * atan2f(e_R_d_length, q.data[0])));

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;

		//e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
		MATHLIB_ASSERT (Vector3f_mul_float (&e_R, (1.0f - direct_w)));
		MATHLIB_ASSERT (Vector3f_init_Vector3f (&v3f_temp1, &e_R_d));
		MATHLIB_ASSERT (Vector3f_mul_float (&v3f_temp1, direct_w));
		MATHLIB_ASSERT (Vector3f_add_Vector3f (&e_R, &v3f_temp1));
	}

	/* calculate angular rates setpoint */
	MATHLIB_ASSERT (Vector3f_emul_Vector3f (&mr_attitude_control_parameters.att_p, &_rates_sp, &e_R));

	/* feed forward yaw setpoint rate */
	_rates_sp.data[2] += yaw_sp_move_rate * yaw_w * mr_attitude_control_parameters.yaw_ff;

	return 0;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
int
mr_attitude_control_control_attitude_rates(float dt)
{
	int i = 0;
	Vector3f v3f_temp1;

	/* reset integral if disarmed */
	if (!_armed.armed) {
		MATHLIB_ASSERT (Vector3f_init_zero (&_rates_int));
	}

	/* current body angular rates */
	Vector3f rates;
	MATHLIB_ASSERT (Vector3f_init_components (&rates, _v_att.roll_rate, _v_att.pitch_rate, _v_att.yaw_rate));

	/* angular rates error */
	Vector3f rates_err;
	MATHLIB_ASSERT (Vector3f_init_Vector3f (&rates_err, &_rates_sp));
	MATHLIB_ASSERT (Vector3f_sub_Vector3f (&rates_err, &rates));

	//_att_control = mr_attitude_control_parameters.rate_p.emult(rates_err) + mr_attitude_control_parameters.rate_d.emult(_rates_prev - rates) / dt + _rates_int;
	MATHLIB_ASSERT (Vector3f_emul_Vector3f (&mr_attitude_control_parameters.rate_p, &_att_control, &rates_err));
	MATHLIB_ASSERT (Vector3f_init_Vector3f (&v3f_temp1, &_rates_prev));
	MATHLIB_ASSERT (Vector3f_sub_Vector3f (&v3f_temp1, &rates));
	MATHLIB_ASSERT (Vector3f_emul_Vector3f (&mr_attitude_control_parameters.rate_d, &v3f_temp1, &v3f_temp1));
	MATHLIB_ASSERT (Vector3f_div_float (&v3f_temp1, dt));
	MATHLIB_ASSERT (Vector3f_sub_Vector3f (&_att_control, &v3f_temp1));
	MATHLIB_ASSERT (Vector3f_sub_Vector3f (&_att_control, &_rates_int));

	MATHLIB_ASSERT (Vector3f_init_Vector3f (&_rates_prev, &rates));

	/* update integral only if not saturated on low limit */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (i = 0; i < 3; i++) {
			if (fabsf(_att_control.data[i]) < _thrust_sp) {
				float rate_i = _rates_int.data[i] + mr_attitude_control_parameters.rate_i.data[i] * rates_err.data[i] * dt;

				if (check_finite(rate_i) && rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				    _att_control.data[i] > -RATES_I_LIMIT && _att_control.data[i] < RATES_I_LIMIT) {
					_rates_int.data[i] = rate_i;
				}
			}
		}
	}

	return 0;
}


int mr_attitude_control_thread_main (void* args)
{
	/* welcome user */
	fprintf (stdout, "Multirotor attitude controller started\n");
	fflush(stdout);

	/* abort on a nonzero return value from the parameter init */
	if (mr_attitude_control_init() != 0) {
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Position estimator aborting on startup due to an error\n");
		exit(-1);
	}

	struct parameter_update_s param_update;
	bool_t updated;

	absolute_time last_run = 0;
	float dt;

	while (!_shutdown_all_systems) {

		/* wait for up to 100ms for data */
		updated = orb_poll(ORB_ID(vehicle_attitude), _v_att_sub, 100);
		if (!updated)
			continue;
		else if (updated < 0)
		{
			/* this is undesirable but not much we can do */
			fprintf (stderr, "Position controller failed to poll global position\n");
			continue;
		}

		dt = (get_absolute_time() - last_run) / 1000000.0f;
		last_run = get_absolute_time();

		/* guard against too small (< 2ms) and too large (> 20ms) dt's */
		if (dt < 0.002f) {
			dt = 0.002f;

		} else if (dt > 0.02f) {
			dt = 0.02f;
		}

		/* copy attitude topic */
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

		/* check for updates in other topics */
		updated = orb_check (ORB_ID(parameter_update), _params_sub);
		if (updated) {
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

			/* update parameters */
			mr_attitude_control_params_update();
		}

		mr_attitude_control_vehicle_control_flags_poll();
		mr_attitude_control_arming_status_poll();
		mr_attitude_control_vehicle_manual_poll();

		if (_v_control_flags.flag_control_attitude_enabled) {
			mr_attitude_control_control_attitude(dt);

			/* publish attitude rates setpoint */
			_v_rates_sp.roll = _rates_sp.data[0];
			_v_rates_sp.pitch = _rates_sp.data[1];
			_v_rates_sp.yaw = _rates_sp.data[2];
			_v_rates_sp.thrust = _thrust_sp;
			_v_rates_sp.timestamp = get_absolute_time();

			orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);


		} else {
			/* attitude controller disabled, poll rates setpoint topic */
			mr_attitude_control_vehicle_rates_setpoint_poll();
			_rates_sp.data[0] = _v_rates_sp.roll;
			_rates_sp.data[1] = _v_rates_sp.pitch;
			_rates_sp.data[2] = _v_rates_sp.yaw;
			_thrust_sp = _v_rates_sp.thrust;
		}

		if (_v_control_flags.flag_control_rates_enabled) {
			mr_attitude_control_control_attitude_rates(dt);

			/* publish actuator controls */
			_actuators.control[0] = (check_finite(_att_control.data[0])) ? _att_control.data[0] : 0.0f;
			_actuators.control[1] = (check_finite(_att_control.data[1])) ? _att_control.data[1] : 0.0f;
			_actuators.control[2] = (check_finite(_att_control.data[2])) ? _att_control.data[2] : 0.0f;
			_actuators.control[3] = (check_finite(_thrust_sp)) ? _thrust_sp : 0.0f;
			//_actuators.timestamp = get_absolute_time();

			orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
		}
	}

	/*
	 * do subscriptions
	 */
	orb_unsubscribe(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude), _v_att_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_control_flags), _v_control_flags_sub, pthread_self());
	orb_unsubscribe(ORB_ID(parameter_update), _params_sub, pthread_self());
	orb_unsubscribe(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, pthread_self());
	orb_unsubscribe(ORB_ID(actuator_armed), _armed_sub, pthread_self());

	/*
	 * publications
	 */
	orb_unadvertise(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, pthread_self());
	orb_unadvertise(ORB_ID(actuator_controls_0), _actuators_0_pub, pthread_self());

	return 0;
}
