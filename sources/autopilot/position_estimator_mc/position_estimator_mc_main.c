/**
 * @file position_estimator_main.c
 * Model-identification based position estimator for multirotors
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <math.h>

#include "../../ORB/ORB.h"
#include "../../ORB/topics/parameter_update.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/sensors/sensor_combined.h"
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/actuator/actuator_armed.h"
#include "../../ORB/topics/position/vehicle_gps_position.h"
#include "../../ORB/topics/position/vehicle_local_position.h"
#include "../../ORB/topics/position/vehicle_global_position.h"

#include "../../uav_library/common.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/math/limits.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"

#include "position_estimator_mc_params.h"
#include "codegen/kalman_dlqe2.h"
#include "codegen/kalman_dlqe2_initialize.h"
#include "codegen/kalman_dlqe3.h"
#include "codegen/kalman_dlqe3_initialize.h"



/* land detector */
absolute_time landed_time;
param_t param_land_time, param_land_alt, param_land_thrust;
float land_t, land_alt, land_thrust;


// WARNING: considering ground level as constant and equal to home_alt
void land_detector (bool_t *landed, float alt, float thrust)
{
	/* detect land */
	float prev_land_status = *landed;
	bool_t _landed = 1;


	if (prev_land_status) {
		if (alt > land_alt && thrust > land_thrust) {
			_landed = 0;
			landed_time = 0;
		}

	} else {
		_landed = 0;

		if (alt < land_alt && thrust < land_thrust) {
			if (landed_time == 0) {
				landed_time = get_absolute_time();    // land detected first time

			} else {
				if (get_absolute_time() > landed_time + land_t * 1000000.0f) {
					_landed = 1;
					landed_time = 0;
				}
			}

		} else {
			landed_time = 0;
		}
	}

	*landed = _landed;
}


int position_estimator_mc_thread_main(int argc, char *argv[])
{
	/* welcome user */
	fprintf (stdout, "Position estimator started\n");
	fflush(stdout);

	/* initialize values */
	float z[3] = {0, 0, 0}; /* output variables from tangent plane mapping */
	// float rotMatrix[4] = {1.0f,  0.0f, 0.0f,  1.0f};
	float x_x_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	float x_y_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	float x_z_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	float x_x_aposteriori[3] = {0.0f, 0.0f, 0.0f};
	float x_y_aposteriori[3] = {1.0f, 0.0f, 0.0f};
	float x_z_aposteriori[3] = {1.0f, 0.0f, 0.0f};

	// XXX this is wrong and should actual dT instead
	const float dT_const_50 = 1.0f/50.0f;

	//computed from dlqe in matlab
	const float K_vicon_50Hz[3] = {0.5297f, 0.9873f, 0.9201f};
	float K[3] = {0.0f, 0.0f, 0.0f};
	int baro_loop_cnt = 0;
	int baro_loop_end = 70; /* measurement for 1 second */
	float p0_Pa = 0.0f; /* to determin while start up */
	float rho0 = 1.293f; /* standard pressure */
	const float const_earth_gravity = 9.81f;

	float posX = 0.0f;
	float posY = 0.0f;
	float posZ = 0.0f;

	double lat, lat_current;
	double lon, lon_current;

	float gps_origin_altitude = 0.0f;

	/* Initialize filter */
	kalman_dlqe2_initialize();
	kalman_dlqe3_initialize();

	/* declare and safely initialize all structs */
	struct parameter_update_s update;
	struct actuator_controls_s actuator;
	memset(&actuator, 0, sizeof(actuator));
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_local_position_s local_pos_est;
	memset(&local_pos_est, 0, sizeof(local_pos_est));
	struct vehicle_global_position_s global_pos_est;
	memset(&global_pos_est, 0, sizeof(global_pos_est));


	/* subscribe */
	orb_subscr_t sub_params = orb_subscribe(ORB_ID(parameter_update));
	if (sub_params < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to parameter_update topic\n");
		exit(-1);
	}

	orb_subscr_t actuator_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	if (actuator_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to actuator_controls topic\n");
		exit(-1);
	}

	orb_subscr_t armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	if (armed_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to actuator_armed topic\n");
		exit(-1);
	}

	orb_subscr_t sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	if (sensor_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to sensor_combined topic\n");
		exit(-1);
	}

	orb_subscr_t vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	if (vehicle_attitude_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to vehicle_attitude topic\n");
		exit(-1);
	}

	orb_subscr_t vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	if (vehicle_gps_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to vehicle_gps_position topic\n");
		exit(-1);
	}


	/* advertise */
	orb_advert_t local_pos_est_pub = orb_advertise(ORB_ID(vehicle_local_position));
	if (local_pos_est_pub == -1)
	{
		fprintf (stderr, "Attitude estimator thread failed to advertise the vehicle_local_position topic\n");
		exit (-1);
	}
	local_pos_est.landed = 1;
	orb_publish (ORB_ID(vehicle_local_position), local_pos_est_pub, &local_pos_est);

	orb_advert_t global_pos_est_pub = orb_advertise(ORB_ID(vehicle_global_position));
	if (global_pos_est_pub == -1)
	{
		fprintf (stderr, "Attitude estimator thread failed to advertise the vehicle_global_position topic\n");
		exit (-1);
	}
	//global_pos_est.landed = landed;
	orb_publish (ORB_ID(vehicle_global_position), global_pos_est_pub, &global_pos_est);

	bool_t updated = 0;
	bool_t gps_updated = 0;
	bool_t flag_use_baro = 0;
	bool_t flag_baro_initialized = 0; /* in any case disable baroINITdone */


	/* initialize parameter handles */
	/* abort on a nonzero return value from the parameter init */
	param_land_time = param_find ("LAND_TIME");
	param_land_alt = param_find ("LAND_ALT");
	param_land_thrust = param_find ("LAND_THRUST");

	if (param_land_time == PARAM_INVALID || param_land_alt == PARAM_INVALID || param_land_thrust == PARAM_INVALID ||
		position_estimator_mc_params_init() != 0)
	{
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Position estimator aborting on startup due to an error\n");
		exit(-1);
	}
	orb_copy(ORB_ID(parameter_update), sub_params, &update); /* read from param to clear updated flag */

	param_get (param_land_time, &land_t);
	param_get (param_land_alt, &land_alt);
	param_get (param_land_thrust, &land_thrust);


	/* wait until gps signal turns valid, only then can we initialize the projection */
	float hdop_threshold_m = 4.0f;
	float vdop_threshold_m = 8.0f;

	/*
	 * If horizontal dilution of precision (hdop / eph)
	 * and vertical diluation of precision (vdop / epv)
	 * are below a certain threshold (e.g. 4 m), AND
	 * home position is not yet set AND the last GPS
	 * GPS measurement is not older than two seconds AND
	 * the system is currently not armed, set home
	 * position to the current position.
	 */

	while (!(gps.fix_type == 3
		&& (gps.eph_m < hdop_threshold_m)
		&& (gps.epv_m < vdop_threshold_m)
		&& (get_absolute_time() - gps.timestamp_position < 2000000))) {

		/* wait for GPS updates, BUT READ VEHICLE STATUS (!)
		 * this choice is critical, since the vehicle status might not
		 * actually change, if this app is started after GPS lock was
		 * aquired.
		 */
		updated = orb_poll(ORB_ID(vehicle_gps_position), vehicle_gps_sub, 10000);
		if (updated) {
			/* Read gps position */
			orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
		}

		updated = orb_check (ORB_ID(parameter_update), sub_params);
		if (updated) {
			/* Read out parameters to check for an update there, e.g. useGPS variable */
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(parameter_update), sub_params, &updated);

			/* update parameters */
			position_estimator_mc_params_update ();
			param_get (param_land_time, &land_t);
			param_get (param_land_alt, &land_alt);
			param_get (param_land_thrust, &land_thrust);
		}
	}

	/* get gps value for first initialization */
	orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);

	lat_current = ((double) (gps.latitude)) * (double) 1e-7;
	lon_current = ((double) (gps.longitude)) * (double) 1e-7;
	gps_origin_altitude = gps.altitude * 1e-3f;

	local_pos_est.ref_lat = gps.latitude;
	local_pos_est.ref_lon = gps.longitude;
	local_pos_est.ref_alt = gps_origin_altitude;	// WARNING: considering ground level as a constant
	local_pos_est.ref_timestamp = get_absolute_time();

	/* initialize coordinates */
	map_projection_init(lat_current, lon_current);

	flag_use_baro = position_estimator_mc_parameters.baro;


	/**< main_loop */
	while (!_shutdown_all_systems) {
		/* wait for up to 20ms for data */
		gps_updated = 0;
		updated = orb_poll(ORB_ID(vehicle_gps_position), vehicle_gps_sub, 20000);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (updated < 0) {
			fprintf (stderr, "Position estimator failed to poll sensor_combined\n");
			continue;
		}
		else if (updated)
		{
			/* new GPS value */
			orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);

			/* Project gps lat lon (Geographic coordinate system) to plane*/
			map_projection_project(((double)(gps.latitude)) * (double) 1e-7, ((double)(gps.longitude)) * (double) 1e-7, &(z[0]), &(z[1]));

			posX = z[0];
			posY = z[1];
			posZ = (float)(gps.altitude * 1e-3f);
			gps_updated = 1;
		}

		updated = orb_check (ORB_ID(parameter_update), sub_params);
		if (updated)
		{
			/* new parameter */
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(parameter_update), sub_params, &updated);

			/* update parameters */
			position_estimator_mc_params_update ();

			flag_use_baro = position_estimator_mc_parameters.baro;
		}

		updated = orb_check (ORB_ID(vehicle_attitude), vehicle_attitude_sub);
		if (updated) {
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
		}

		/* actuator */
		updated = orb_check (ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub);
		if (updated) {
			orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, &actuator);
		}

		/* armed */
		updated = orb_check (ORB_ID(actuator_armed), armed_sub);
		if (updated) {
			orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
		}

		/* Main estimator loop */
		updated = orb_check (ORB_ID(sensor_combined), sensor_sub);
		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensor);
		}

		// barometric pressure estimation at start up
		if (!flag_baro_initialized){
			// mean calculation over several measurements
			if (baro_loop_cnt<baro_loop_end) {
				p0_Pa += (sensor.baro_pres_mbar*100);
				baro_loop_cnt++;
			} else {
				p0_Pa /= (float)(baro_loop_cnt);
				flag_baro_initialized = 1;
			}
		}


		/* initialize map projection with the last estimate (not at full rate) */
		if (gps.fix_type > 2) {
			/* x-y-position/velocity estimation in earth frame = gps frame */
			kalman_dlqe3(dT_const_50,K_vicon_50Hz[0],K_vicon_50Hz[1],K_vicon_50Hz[2],x_x_aposteriori_k,posX,gps_updated,0.0f,0.0f,x_x_aposteriori);
			memcpy(x_x_aposteriori_k, x_x_aposteriori, sizeof(x_x_aposteriori));
			kalman_dlqe3(dT_const_50,K_vicon_50Hz[0],K_vicon_50Hz[1],K_vicon_50Hz[2],x_y_aposteriori_k,posY,gps_updated,0.0f,0.0f,x_y_aposteriori);
			memcpy(x_y_aposteriori_k, x_y_aposteriori, sizeof(x_y_aposteriori));
			/* z-position/velocity estimation in earth frame = vicon frame */
			float z_est = 0.0f;
			if (flag_baro_initialized && flag_use_baro) {
				z_est = -p0_Pa*logf(p0_Pa/(sensor.baro_pres_mbar*100))/(rho0*const_earth_gravity);
				K[0] = K_vicon_50Hz[0];
				K[1] = K_vicon_50Hz[1];
				K[2] = K_vicon_50Hz[2];
				gps_updated = 1.0f; /* always enable the update, cause baro update = 200 Hz */
			} else {
				z_est = posZ;
				K[0] = K_vicon_50Hz[0];
				K[1] = K_vicon_50Hz[1];
				K[2] = K_vicon_50Hz[2];
			}

			kalman_dlqe3(dT_const_50,K[0],K[1],K[2],x_z_aposteriori_k,z_est,gps_updated,0.0f,0.0f,x_z_aposteriori);
			memcpy(x_z_aposteriori_k, x_z_aposteriori, sizeof(x_z_aposteriori));

			local_pos_est.timestamp = get_absolute_time();
			local_pos_est.x = x_x_aposteriori_k[0];
			local_pos_est.vx = x_x_aposteriori_k[1];
			local_pos_est.y = x_y_aposteriori_k[0];
			local_pos_est.vy = x_y_aposteriori_k[1];
			local_pos_est.z = -(x_z_aposteriori_k[0] - gps_origin_altitude);
			local_pos_est.vz = x_z_aposteriori_k[1];
			local_pos_est.yaw = att.yaw;
			land_detector (&local_pos_est.landed, -local_pos_est.z, (armed.armed)? actuator.control[3] : 0.0f);

			if ((check_finite(x_x_aposteriori_k[0])) && (check_finite(x_x_aposteriori_k[1])) && (check_finite(x_y_aposteriori_k[0])) &&
				(check_finite(x_y_aposteriori_k[1])) && (check_finite(x_z_aposteriori_k[0])) && (check_finite(x_z_aposteriori_k[1]))) {

				local_pos_est.xy_valid = 1;
				local_pos_est.z_valid = 1;
				local_pos_est.xy_global = 1;
				local_pos_est.z_global = 1;
				local_pos_est.v_xy_valid = 1;
				local_pos_est.v_z_valid = 1;

				/* publish local position estimate */
				orb_publish(ORB_ID(vehicle_local_position), local_pos_est_pub, &local_pos_est);
			}
			else
			{
				local_pos_est.xy_valid = 0;
				local_pos_est.z_valid = 0;
				local_pos_est.xy_global = 0;
				local_pos_est.z_global = 0;
				local_pos_est.v_xy_valid = 0;
				local_pos_est.v_z_valid = 0;
			}

			/* publish on GPS updates */
			if (gps_updated) {
				map_projection_reproject(local_pos_est.x, local_pos_est.y, &lat, &lon);

				global_pos_est.valid = local_pos_est.xy_valid && local_pos_est.z_valid;
				global_pos_est.latitude = lat*1e7;
				global_pos_est.longitude = lon*1e7;
				global_pos_est.altitude = z_est;
				global_pos_est.relative_altitude = z_est - gps_origin_altitude;
				global_pos_est.ground_level = local_pos_est.ref_alt;
				global_pos_est.time_gps_usec = gps.time_gps_usec;

				/* set valid values even if position is not valid */
				if (local_pos_est.v_xy_valid) {
					global_pos_est.vx = local_pos_est.vx;
					global_pos_est.vy = local_pos_est.vy;
				}
				if (local_pos_est.v_z_valid) {
					global_pos_est.vz = local_pos_est.vz;
				}

				global_pos_est.yaw = local_pos_est.yaw;
				global_pos_est.landed = local_pos_est.landed;

				orb_publish(ORB_ID(vehicle_global_position), global_pos_est_pub, &global_pos_est);
			}
		}
	}


	/*
	 * do unsubscriptions
	 */
	orb_unsubscribe(ORB_ID(parameter_update), sub_params, pthread_self());
	orb_unsubscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, pthread_self());
	orb_unsubscribe(ORB_ID(actuator_armed), armed_sub, pthread_self());
	orb_unsubscribe(ORB_ID(sensor_combined), sensor_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_gps_position), vehicle_gps_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude), vehicle_attitude_sub, pthread_self());

	/*
	 * do unadvertises
	 */
	orb_unadvertise(ORB_ID(vehicle_local_position), local_pos_est_pub, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_global_position), global_pos_est_pub, pthread_self());

	return 0;
}
