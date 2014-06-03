/*
 * Author: Hyon Lim <limhyon@gmail.com, hyonlim@snu.ac.kr>
 *
 * @file attitude_estimator_so3_comp_main.c
 *
 * Implementation of nonlinear complementary filters on the SO(3).
 * This code performs attitude estimation by using accelerometer, gyroscopes and magnetometer.
 * Result is provided as quaternion, 1-2-3 Euler angle and rotation matrix.
 * 
 * Theory of nonlinear complementary filters on the SO(3) is based on [1].
 * Quaternion realization of [1] is based on [2].
 * Optmized quaternion update code is based on Sebastian Madgwick's implementation.
 * 
 * References
 *  [1] Mahony, R.; Hamel, T.; Pflimlin, Jean-Michel, "Nonlinear Complementary Filters on the Special Orthogonal Group," Automatic Control, IEEE Transactions on , vol.53, no.5, pp.1203,1218, June 2008
 *  [2] Euston, M.; Coote, P.; Mahony, R.; Jonghyuk Kim; Hamel, T., "A complementary filter for attitude estimation of a fixed-wing UAV," Intelligent Robots and Systems, 2008. IROS 2008. IEEE/RSJ International Conference on , vol., no., pp.340,345, 22-26 Sept. 2008
 */


#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


#include "../../ORB/ORB.h"
#include "../../ORB/topics/sensors/sensor_combined.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/parameter_update.h"

#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/math/limits.h"
#include "../../uav_library/math/EulerAngles.h"
#include "../../uav_library/math/Quaternion.h"
#include "../../uav_library/math/Dcm.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/common.h"

#include "attitude_estimator_so3_comp_main.h"
#include "attitude_estimator_so3_comp_params.h"


static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
static bool_t bFilterInit = 0 /* false */;

//! Auxiliary variables to reduce number of repeated operations
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;



/**
 *  Fast inverse square-root
 *  See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
float invSqrt(float number) {
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    //i = (long) y;
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    //y = (float) i;
    y = y * ( f - ( x * y * y ) );
    return y;
}

/**
 * Using accelerometer, sense the gravity vector.
 * Using magnetometer, sense yaw.
 */
void non_linear_SO3_AHRS_init(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}


void non_linear_SO3_AHRS_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt) {
	float recipNorm;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

	//! Make filter converge to initial solution faster
	//! This function assumes you are in static position.
	//! WARNING : in case air reboot, this can cause problem. But this is very
	//!	      unlikely happen.
	if(bFilterInit == 0 /* false */)
	{
		non_linear_SO3_AHRS_init(ax,ay,az,mx,my,mz);
		bFilterInit = 1 /* true */;
	}
        	
	//! If magnetometer measurement is available, use it.
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		float hx, hy, hz, bx, bz;
		float halfwx, halfwy, halfwz;
	
		// Normalise magnetometer measurement
		// Will sqrt work better? PX4 system is powerful enough?
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 - q2q2);
		bx = sqrt(hx * hx + hy * hy);
		bz = hz;

		// Estimated direction of magnetic field
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (my * halfwz - mz * halfwy);
		halfey += (mz * halfwx - mx * halfwz);
		halfez += (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float halfvx, halfvy, halfvz;
	
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += ay * halfvz - az * halfvy;
		halfey += az * halfvx - ax * halfvz;
		halfez += ax * halfvy - ay * halfvx;
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
			gyro_bias[1] += twoKi * halfey * dt;
			gyro_bias[2] += twoKi * halfez * dt;
			gx += gyro_bias[0];	// apply integral feedback
			gy += gyro_bias[1];
			gz += gyro_bias[2];
		}
		else {
			gyro_bias[0] = 0.0f;	// prevent integral windup
			gyro_bias[1] = 0.0f;
			gyro_bias[2] = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	//! Integrate rate of change of quaternion
#if 0
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
#endif 

	// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
	//! q_k = q_{k-1} + dt*\dot{q}
	//! \dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

	q0 += dt*dq0;
	q1 += dt*dq1;
	q2 += dt*dq2;
	q3 += dt*dq3;
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
}


/*
 * [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
 */

/*
 * Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 */
void* attitude_estimator_so3_comp_thread_main (void* args)
{
	//! Time constant
	float dt = 0.005f;
	
	Quaternion q;
	Dcm Rot_matrix;
	EulerAngles euler;
	float euler0, euler1, euler2;

	Dcm_init_default (&Rot_matrix);
	Quaternion_init_default (&q);	/**< init: identity matrix */
	EulerAngles_init_default (&euler);	/* output euler angles */


	float acc[3] = {0.0f, 0.0f, 0.0f};
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};


	/* welcome user */
	fprintf (stdout, "Attitude Estimator started\n");
	fflush(stdout);


	/* store start time to guard against too slow update rates */
	absolute_time start_time = get_absolute_time();
	absolute_time last_measurement = 0;
	//absolute_time last_data = 0;

	struct parameter_update_s update;
	struct sensor_combined_s raw;
	struct vehicle_attitude_s att;
	memset(&update, 0, sizeof(update));
	memset(&raw, 0, sizeof(raw));
	memset(&att, 0, sizeof(att));


	/* subscribe to raw data */
	orb_subscr_t sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	if (sub_raw == -1)
	{
		fprintf (stderr, "Attitude estimator thread failed to subscribe to the sensor_combined topic\n");
		exit (-1);
	}

	/* subscribe to param changes */
	orb_subscr_t sub_params = orb_subscribe(ORB_ID(parameter_update));
	if (sub_params == -1)
	{
		fprintf (stderr, "Attitude estimator thread failed to subscribe to the parameter_update topic\n");
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


	/* keep track of sensor updates */
	//float sensor_update_hz[3] = {0.0f, 0.0f, 0.0f};
	//uint8_t update_vect[3] = {0, 0, 0};
	//absolute_time sensor_last_timestamp[3] = {0, 0, 0};
	uint32_t sensor_last_count[3] = {0, 0, 0};


	/* abort on a nonzero return value from the parameter init */
	if (non_linear_SO3_AHRS_comp_params_init() != 0) {
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Attitude estimator aborting on startup due to an error\n");
		exit(-1);
	}


	int updated;
	absolute_time usec_max_poll_wait_time = 1000000;
	bool_t initialized = 0 /* false */, sensors_found_once = 0 /* false */;
	//bool_t  const_initialized = 0 /* false */;

	float gyro_offsets[3] = { 0.0f, 0.0f, 0.0f };
	unsigned offset_count = 0;


	/* Main loop*/
	while (!_shutdown_all_systems) {
		updated = orb_check(ORB_ID(parameter_update), sub_params);
		if (updated) {
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(parameter_update), sub_params, &update);

			/* update parameters */
			non_linear_SO3_AHRS_comp_params_update();
		}

		/* wait for up to 1000ms for data */
		updated = orb_poll(ORB_ID(sensor_combined), sub_raw, usec_max_poll_wait_time);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (!updated && sensors_found_once && getenv("VERY_VERBOSE"))
		{
			fprintf (stderr, "WARNING: Attitude estimator not getting sensors\n");
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (updated < 0)
		{
			fprintf (stderr, "Attitude estimator failed to poll sensors\n");
			continue;
		}


		/* get latest measurements */
		orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
		sensors_found_once = 1;

		if (!initialized)
		{
			gyro_offsets[0] += raw.gyro_rad_s[0];
			gyro_offsets[1] += raw.gyro_rad_s[1];
			gyro_offsets[2] += raw.gyro_rad_s[2];
			offset_count++;

			if (get_absolute_time() - start_time > 3000000LL) {
				initialized = 1 /* true */;
				gyro_offsets[0] /= offset_count;
				gyro_offsets[1] /= offset_count;
				gyro_offsets[2] /= offset_count;
			}

		}
		else
		{
			/* Calculate data time difference in seconds */
			dt = (raw.timestamp - last_measurement) / 1000000.0f;
			last_measurement = raw.timestamp;

			/* Fill in gyro measurements */
			if (sensor_last_count[0] != raw.gyro_counter) {
				//update_vect[0] = 1;
				//sensor_update_hz[0] = 1e6f / (raw.timestamp - sensor_last_timestamp[0]);
				//sensor_last_timestamp[0] = raw.timestamp;
				sensor_last_count[0] = raw.gyro_counter;
			}

			gyro[0] =  raw.gyro_rad_s[0] - gyro_offsets[0];
			gyro[1] =  raw.gyro_rad_s[1] - gyro_offsets[1];
			gyro[2] =  raw.gyro_rad_s[2] - gyro_offsets[2];

			/* update accelerometer measurements */
			if (sensor_last_count[1] != raw.accelerometer_counter) {
				//update_vect[1] = 1;
				//sensor_update_hz[1] = 1e6f / (raw.timestamp - sensor_last_timestamp[1]);
				//sensor_last_timestamp[1] = raw.timestamp;
				sensor_last_count[1] = raw.accelerometer_counter;
			}

			acc[0] = raw.accelerometer_m_s2[0];
			acc[1] = raw.accelerometer_m_s2[1];
			acc[2] = raw.accelerometer_m_s2[2];

			/* update magnetometer measurements */
			if (sensor_last_count[2] != raw.magnetometer_counter) {
				//update_vect[2] = 1;
				//sensor_update_hz[2] = 1e6f / (raw.timestamp - sensor_last_timestamp[2]);
				//sensor_last_timestamp[2] = raw.timestamp;
				sensor_last_count[2] = raw.magnetometer_counter;
			}

			mag[0] = raw.magnetometer_ga[0];
			mag[1] = raw.magnetometer_ga[1];
			mag[2] = raw.magnetometer_ga[2];


			/* initialize with good values once we have a reasonable dt estimate */
			/* do not execute the filter if not initialized */
			/*if (!const_initialized && |check_out_of_bound (dt, , )) {
				const_initialized = (non_linear_SO3_AHRS_comp_params_update () == 0)? 1 : 0;
			}

			if (!const_initialized) {
				continue;
			}*/

			// NOTE : Accelerometer is reversed.
			// Because proper mount of PX4 will give you a reversed accelerometer readings.
			non_linear_SO3_AHRS_update (gyro[0],gyro[1],gyro[2],-acc[0],-acc[1],-acc[2],mag[0],mag[1],mag[2],so3_comp_params.Kp,so3_comp_params.Ki, dt);


			Quaternion_init_components (&q, q0, q1, q2, q3);

			// Convert q->R, This R converts inertial frame to body frame.
			Dcm_init_Quaternion (&Rot_matrix, &q);

			//1-2-3 Representation.
			//Equation (290)
			//Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
			// Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
			EulerAngles_init_Dcm (&euler, &Rot_matrix);
			EulerAngles_getPhi (&euler, &euler0);
			EulerAngles_getTheta (&euler, &euler1);
			EulerAngles_getPsi (&euler, &euler2);

			/* swap values for next iteration, check for fatal inputs */
			if (check_out_of_bounds(euler0, -M_PI, M_PI) ||
				check_out_of_bounds(euler1, -M_PI_2, M_PI_2) ||
				check_out_of_bounds(euler2, -M_PI, M_PI))
			{
				// due to inputs or numerical failure the output is invalid, skip it
				fprintf (stderr, "Attitude estimator computed a NaN in Euler angles estimate\n");
				continue;
			}

			//if (last_data > 0 && raw.timestamp - last_data > 50000)
				//fprintf (stderr, "Attitude estimator missed sensor data\n");

			//last_data = raw.timestamp;

			/* send out */
			// XXX Apply the same transformation to the rotation matrix
			att.roll = euler0 - so3_comp_params.roll_off;
			att.pitch = euler1 - so3_comp_params.pitch_off;
			att.yaw = _wrap_pi(euler2 - so3_comp_params.yaw_off);


			//! Euler angle rate. But it needs to be investigated again.
			/*
			att.roll_rate = 2.0f*(-q1*dq0 + q0*dq1 - q3*dq2 + q2*dq3);
			att.pitch_rate = 2.0f*(-q2*dq0 + q3*dq1 + q0*dq2 - q1*dq3);
			att.yaw_rate = 2.0f*(-q3*dq0 -q2*dq1 + q1*dq2 + q0*dq3);
			*/
			att.roll_rate = gyro[0];
			att.pitch_rate = gyro[1];
			att.yaw_rate = gyro[2];

			att.roll_acc = 0;
			att.pitch_acc = 0;
			att.yaw_acc = 0;

			//! Quaternion
			att.q[0] = q0;
			att.q[1] = q1;
			att.q[2] = q2;
			att.q[3] = q3;
			att.q_valid = 1 /* true */;

			/* TODO: Bias estimation required */
			memcpy(&att.rate_offsets, &(gyro_bias), sizeof(att.rate_offsets));

			/* copy rotation matrix */
			Dcm_get_data(&Rot_matrix, &att.R[0][0], 0, 0);
			Dcm_get_data(&Rot_matrix, &att.R[0][1], 0, 1);
			Dcm_get_data(&Rot_matrix, &att.R[0][2], 0, 2);
			Dcm_get_data(&Rot_matrix, &att.R[1][0], 1, 0);
			Dcm_get_data(&Rot_matrix, &att.R[1][1], 1, 1);
			Dcm_get_data(&Rot_matrix, &att.R[1][2], 1, 2);
			Dcm_get_data(&Rot_matrix, &att.R[2][0], 2, 0);
			Dcm_get_data(&Rot_matrix, &att.R[2][1], 2, 1);
			Dcm_get_data(&Rot_matrix, &att.R[2][2], 2, 2);
			att.R_valid = 1 /* true */;


			// Broadcast
			orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
		}
	}


	// *********************************** unsubscribe and unadvertise ****************************************
	orb_unsubscribe (ORB_ID(parameter_update), sub_params, pthread_self());
	orb_unsubscribe (ORB_ID(sensor_combined), sub_raw, pthread_self());

	orb_unadvertise (ORB_ID(vehicle_attitude), pub_att, pthread_self());

	return 0;
}
