/**
 * @file position_estimator_inav_main.c
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
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/actuator/actuator_armed.h"
#include "../../ORB/topics/sensors/sensor_combined.h"
#include "../../ORB/topics/sensors/sensor_optical_flow.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/position/vehicle_local_position.h"
#include "../../ORB/topics/position/vehicle_global_position.h"
#include "../../ORB/topics/position/vehicle_gps_position.h"

#include "../../uav_library/common.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/math/limits.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/geo/geo.h"

#include "position_estimator_inav_main.h"
#include "position_estimator_inav_params.h"
#include "inertial_filter.h"


#define MIN_VALID_W 0.00001f

static const absolute_time gps_topic_timeout = 1000000;		// GPS topic timeout = 1s
static const absolute_time flow_topic_timeout = 1000000;	// optical flow topic timeout = 1s
static const absolute_time sonar_timeout = 150000;	// sonar timeout = 150ms
static const absolute_time sonar_valid_timeout = 1000000;	// estimate sonar distance during this time after sonar loss
static const absolute_time xy_src_timeout = 2000000;	// estimate position during this time after position sources loss
static const uint32_t pub_interval = 10000;	// limit publish rate to 100 Hz
static const float max_flow = 1.0f;	// max flow value that can be used, rad/s


void *position_estimator_inav_thread_main (void* args)
{
	/* welcome user */
	fprintf (stdout, "Position estimator started\n");
	fflush(stdout);

	//int mavlink_fd;
	//mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	//mavlink_log_info(mavlink_fd, "[inav] started");

	int i = 0, j = 0;
	float x_est[3] = { 0.0f, 0.0f, 0.0f };
	float y_est[3] = { 0.0f, 0.0f, 0.0f };
	float z_est[3] = { 0.0f, 0.0f, 0.0f };

	int baro_init_cnt = 0;
	int baro_init_num = 200;
	float baro_offset = 0.0f;		// baro offset for reference altitude, initialized on start, then adjusted
	float surface_offset = 0.0f;	// ground level offset from reference altitude
	float surface_offset_rate = 0.0f;	// surface offset change rate
	float alt_avg = 0.0f;
	bool_t landed = 1;
	absolute_time landed_time = 0;

	uint32_t accel_counter = 0;
	uint32_t baro_counter = 0;

	bool_t ref_inited = 0;
	absolute_time ref_init_start = 0;
	const absolute_time ref_init_delay = 1000000;	// wait for 1s after 3D fix

	uint16_t accel_updates = 0;
	uint16_t baro_updates = 0;
	uint16_t gps_updates = 0;
	uint16_t attitude_updates = 0;
	uint16_t flow_updates = 0;

	absolute_time pub_last = get_absolute_time();
	absolute_time t_prev = 0;

	/* acceleration in NED frame */
	float accel_NED[3] = { 0.0f, 0.0f, -CONSTANTS_ONE_G };

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	float corr_acc[] = { 0.0f, 0.0f, 0.0f };	// N E D
	float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame
	float corr_baro = 0.0f;		// D
	float corr_gps[3][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};
	float w_gps_xy = 1.0f;
	float w_gps_z = 1.0f;
	float corr_sonar = 0.0f;
	float corr_sonar_filtered = 0.0f;

	float corr_flow[] = { 0.0f, 0.0f };	// N E
	float w_flow = 0.0f;

	float sonar_prev = 0.0f;
	absolute_time sonar_time = 0;			// time of last sonar measurement (not filtered)
	absolute_time sonar_valid_time = 0;	// time of last sonar measurement used for correction (filtered)
	absolute_time xy_src_time = 0;		// time of last available position data

	bool_t updated;
	bool_t wait_baro = 1;		// wait for initial baro value
	bool_t gps_valid = 0;			// GPS is valid
	bool_t sonar_valid = 0;		// sonar is valid
	bool_t flow_valid = 0;		// flow is valid
	bool_t flow_accurate = 0;		// flow should be accurate (this flag not updated if flow_valid == 0)

	/* declare and safely initialize all structs */
	struct parameter_update_s p_update;
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
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct sensor_optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));


	/* subscribe */
	orb_subscr_t parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	if (parameter_update_sub < 0)
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

	orb_subscr_t sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	if (sensor_combined_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to sensor_combined topic\n");
		exit(-1);
	}

	orb_subscr_t optical_flow_sub = orb_subscribe(ORB_ID(sensor_optical_flow));
	if (optical_flow_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to sensor_optical_flow topic\n");
		exit(-1);
	}

	orb_subscr_t vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	if (vehicle_attitude_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to vehicle_attitude topic\n");
		exit(-1);
	}

	orb_subscr_t vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	if (vehicle_gps_position_sub < 0)
	{
		fprintf (stderr, "Position estimator thread failed to subscribe to vehicle_gps_position topic\n");
		exit(-1);
	}


	/* advertise */
	orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position));
	if (vehicle_local_position_pub == -1)
	{
		fprintf (stderr, "Attitude estimator thread failed to advertise the vehicle_local_position topic\n");
		exit (-1);
	}
	//local_pos.landed = landed;
	orb_publish (ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);

	orb_advert_t vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position));
	if (vehicle_global_position_pub == -1)
	{
		fprintf (stderr, "Attitude estimator thread failed to advertise the vehicle_global_position topic\n");
		exit (-1);
	}
	//global_pos.landed = landed;
	orb_publish (ORB_ID(vehicle_global_position), vehicle_global_position_pub, &global_pos);


	/* abort on a nonzero return value from the parameter init */
	if (position_estimator_inav_params_init() != 0) {
		/* parameter setup went wrong, abort */
		fprintf (stderr, "Position estimator aborting on startup due to an error\n");
		exit(-1);
	}

	absolute_time t;

	while (wait_baro && !_shutdown_all_systems) {
		/* wait for up to 1000ms for data */
		updated = orb_poll(ORB_ID(sensor_combined), sensor_combined_sub, 1000000);

		/* timed out - periodic check for _shutdown_all_systems, etc. */
		if (!updated && getenv("VERY_VERBOSE"))
		{
			fprintf (stderr, "WARNING: Position estimator not getting sensors\n");
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (updated < 0) {
			fprintf (stderr, "Position estimator failed to poll sensor_combined\n");
			continue;
		}

		/* load local copies */
		orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

		if (wait_baro && sensor.baro_counter > baro_counter) {
			baro_counter = sensor.baro_counter;

			/* mean calculation over several measurements */
			if (baro_init_cnt < baro_init_num) {
				baro_offset += sensor.baro_alt_meter;
				baro_init_cnt++;

			} else {
				wait_baro = 0 /* 0 */;
				baro_offset /= (float) baro_init_cnt;

				//warnx("init baro: alt = %.3f", baro_alt0);
				////mavlink_log_info(mavlink_fd, "[inav] init baro: alt = %.3f", baro_alt0);

				local_pos.z_valid = 1;
				local_pos.v_z_valid = 1;
			}
		}
	}


	while (!_shutdown_all_systems) {

		/* vehicle attitude */
		updated = orb_poll (ORB_ID(vehicle_attitude), vehicle_attitude_sub, 15000);
		t = get_absolute_time();

		if (updated) {
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			attitude_updates++;
		
			/* parameter update */
			updated = orb_check (ORB_ID(parameter_update), parameter_update_sub);
			if (updated) {
				/* read from param to clear updated flag */
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &p_update);

				/* update parameters */
				position_estimator_inav_params_update ();
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

			/* sensor combined */
			updated = orb_check (ORB_ID(sensor_combined), sensor_combined_sub);
			if (updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (sensor.accelerometer_counter != accel_counter) {
					if (att.R_valid) {
						/* correct accel bias */
						sensor.accelerometer_m_s2[0] -= acc_bias[0];
						sensor.accelerometer_m_s2[1] -= acc_bias[1];
						sensor.accelerometer_m_s2[2] -= acc_bias[2];

						/* transform acceleration vector from body frame to NED frame */
						for (i = 0; i < 3; i++) {
							accel_NED[i] = 0.0f;

							for (j = 0; j < 3; j++) {
								accel_NED[i] += att.R[i][j] * sensor.accelerometer_m_s2[j];
							}
						}

						corr_acc[0] = accel_NED[0] - x_est[2];
						corr_acc[1] = accel_NED[1] - y_est[2];
						corr_acc[2] = accel_NED[2] + CONSTANTS_ONE_G - z_est[2];

					} else {
						memset(corr_acc, 0, sizeof(corr_acc));
					}

					accel_counter = sensor.accelerometer_counter;
					accel_updates++;
				}

				if (sensor.baro_counter != baro_counter) {
					corr_baro = baro_offset - sensor.baro_alt_meter - z_est[0];
					baro_counter = sensor.baro_counter;
					baro_updates++;
				}
			}

			
		}

		/* optical flow */
		updated = orb_check (ORB_ID(sensor_optical_flow), optical_flow_sub);
		if (updated) {
			orb_copy(ORB_ID(sensor_optical_flow), optical_flow_sub, &flow);

			if (flow.ground_distance_m > 0.31f && flow.ground_distance_m < 4.0f && att.R[2][2] > 0.7 && flow.ground_distance_m != sonar_prev) {
				sonar_time = t;
				sonar_prev = flow.ground_distance_m;
				corr_sonar = flow.ground_distance_m + surface_offset + z_est[0];
				corr_sonar_filtered += (corr_sonar - corr_sonar_filtered) * position_estimator_inav_parameters.sonar_filt;

				if (fabsf(corr_sonar) > position_estimator_inav_parameters.sonar_err) {
					/* correction is too large: spike or new ground level? */
					if (fabsf(corr_sonar - corr_sonar_filtered) > position_estimator_inav_parameters.sonar_err) {
						/* spike detected, ignore */
						corr_sonar = 0.0f;
						sonar_valid = 0;

					} else {
						/* new ground level */
						surface_offset -= corr_sonar;
						surface_offset_rate = 0.0f;
						corr_sonar = 0.0f;
						corr_sonar_filtered = 0.0f;
						sonar_valid_time = t;
						sonar_valid = 1;
						local_pos.surface_bottom_timestamp = t;
					}

				} else {
					/* correction is ok, use it */
					sonar_valid_time = t;
					sonar_valid = 1;
				}
			}
			
			float flow_q = flow.quality / 255.0f;
			float dist_bottom = - z_est[0] - surface_offset;

			// unused (only ground_distance is set in flow)
			if (dist_bottom > 0.3f && flow_q > position_estimator_inav_parameters.flow_q_min && (t < sonar_valid_time + sonar_valid_timeout) && att.R[2][2] > 0.7) {
				/* distance to surface */
				float flow_dist = dist_bottom / att.R[2][2];
				/* check if flow if too large for accurate measurements */
				/* calculate estimated velocity in body frame */
				float body_v_est[2] = { 0.0f, 0.0f };

				for (i = 0; i < 2; i++) {
					body_v_est[i] = att.R[0][i] * x_est[1] + att.R[1][i] * y_est[1] + att.R[2][i] * z_est[1];
				}

				/* set this flag if flow should be accurate according to current velocity and attitude rate estimate */
				flow_accurate = fabsf(body_v_est[1] / flow_dist - att.roll_rate) < max_flow &&
						fabsf(body_v_est[0] / flow_dist + att.pitch_rate) < max_flow;

				/* convert raw flow to angular flow */
				float flow_ang[2];
				flow_ang[0] = flow.flow_raw_x * position_estimator_inav_parameters.flow_k;
				flow_ang[1] = flow.flow_raw_y * position_estimator_inav_parameters.flow_k;
				/* flow measurements vector */
				float flow_m[3];
				flow_m[0] = -flow_ang[0] * flow_dist;
				flow_m[1] = -flow_ang[1] * flow_dist;
				flow_m[2] = z_est[1];
				/* velocity in NED */
				float flow_v[2] = { 0.0f, 0.0f };

				/* project measurements vector to NED basis, skip Z component */
				for (i = 0; i < 2; i++) {
					for (j = 0; j < 3; j++) {
						flow_v[i] += att.R[i][j] * flow_m[j];
					}
				}

				/* velocity correction */
				corr_flow[0] = flow_v[0] - x_est[1];
				corr_flow[1] = flow_v[1] - y_est[1];
				/* adjust correction weight */
				float flow_q_weight = (flow_q - position_estimator_inav_parameters.flow_q_min) / (1.0f - position_estimator_inav_parameters.flow_q_min);
				w_flow = att.R[2][2] * flow_q_weight / fmaxf(1.0f, flow_dist);

				/* if flow is not accurate, reduce weight for it */
				// TODO make this more fuzzy
				if (!flow_accurate)
					w_flow *= 0.05f;

				flow_valid = 1;

			} else {
				w_flow = 0.0f;
				flow_valid = 0;
			}

			flow_updates++;
		}

		/* vehicle GPS position */
		updated = orb_check(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub);
		if (updated) {
			orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);

			if (gps.fix_type >= 3) {
				/* hysteresis for GPS quality */
				if (gps_valid) {
					if (gps.eph_m > 10.0f || gps.epv_m > 20.0f) {
						gps_valid = 0;
						//mavlink_log_info(mavlink_fd, "[inav] GPS signal lost");
					}

				} else {
					if (gps.eph_m < 5.0f && gps.epv_m < 10.0f) {
						gps_valid = 1;
						//mavlink_log_info(mavlink_fd, "[inav] GPS signal found");
					}
				}

			} else {
				gps_valid = 0;
			}

			if (gps_valid) {
				/* initialize reference position if needed */
				if (!ref_inited) {
					if (ref_init_start == 0) {
						ref_init_start = t;

					} else if (t > ref_init_start + ref_init_delay) {
						ref_inited = 1;
						/* reference GPS position */
						double lat = gps.latitude * 1e-7;
						double lon = gps.longitude * 1e-7;
						float alt = gps.altitude * 1e-3;

						local_pos.ref_lat = gps.latitude;
						local_pos.ref_lon = gps.longitude;
						local_pos.ref_alt = alt + z_est[0];
						local_pos.ref_timestamp = t;

						/* initialize projection */
						map_projection_init(lat, lon);
						//warnx("init ref: lat=%.7f, lon=%.7f, alt=%.2f", lat, lon, alt);
						//mavlink_log_info(mavlink_fd, "[inav] init ref: lat=%.7f, lon=%.7f, alt=%.2f", lat, lon, alt);
					}
				}

				if (ref_inited) {
					/* project GPS lat lon to plane */
					float gps_proj[2];
					map_projection_project(gps.latitude * 1e-7, gps.longitude * 1e-7, &(gps_proj[0]), &(gps_proj[1]));
					/* calculate correction for position */
					corr_gps[0][0] = gps_proj[0] - x_est[0];
					corr_gps[1][0] = gps_proj[1] - y_est[0];
					corr_gps[2][0] = local_pos.ref_alt - gps.altitude * 1e-3 - z_est[0];

					/* calculate correction for velocity */
					if (gps.vel_ned_valid) {
						corr_gps[0][1] = gps.vel_n_m_s - x_est[1];
						corr_gps[1][1] = gps.vel_e_m_s - y_est[1];
						corr_gps[2][1] = gps.vel_d_m_s - z_est[1];

					} else {
						corr_gps[0][1] = 0.0f;
						corr_gps[1][1] = 0.0f;
						corr_gps[2][1] = 0.0f;
					}

					w_gps_xy = 2.0f / fmaxf(2.0f, gps.eph_m);
					w_gps_z = 4.0f / fmaxf(4.0f, gps.epv_m);
				}

			} else {
				/* no GPS lock */
				memset(corr_gps, 0, sizeof(corr_gps));
				ref_init_start = 0;
			}

			gps_updates++;
		}
		else if (getenv("VERY_VERBOSE"))
		{
			fprintf (stderr, "WARNING: Position estimator not getting gps\n");
			continue;
		}


		/* check for timeout on FLOW topic */
		if ((flow_valid || sonar_valid) && t > flow.timestamp + flow_topic_timeout) {
			flow_valid = 0;
			sonar_valid = 0;
			//warnx("FLOW timeout");
			//mavlink_log_info(mavlink_fd, "[inav] FLOW timeout");
		}

		/* check for timeout on GPS topic */
		if (gps_valid && t > gps.timestamp_position + gps_topic_timeout) {
			gps_valid = 0;
			//warnx("GPS timeout");
			//mavlink_log_info(mavlink_fd, "[inav] GPS timeout");
		}

		/* check for sonar measurement timeout */
		if (sonar_valid && t > sonar_time + sonar_timeout) {
			corr_sonar = 0.0f;
			sonar_valid = 0;
		}

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.02, dt), 0.002);		// constrain dt from 2 to 20 ms
		dt = 0.02f;
		t_prev = t;

		/* use GPS if it's valid and reference position initialized */
		bool_t use_gps_xy = ref_inited && gps_valid && position_estimator_inav_parameters.w_xy_gps_p > MIN_VALID_W;
		bool_t use_gps_z = ref_inited && gps_valid && position_estimator_inav_parameters.w_z_gps_p > MIN_VALID_W;
		/* use flow if it's valid and (accurate or no GPS available) */
		bool_t use_flow = flow_valid && (flow_accurate || !use_gps_xy);

		/* try to estimate position during some time after position sources lost */
		if (use_gps_xy || use_flow) {
			xy_src_time = t;
		}

		bool_t can_estimate_xy = (t < xy_src_time + xy_src_timeout);
		bool_t dist_bottom_valid = (t < sonar_valid_time + sonar_valid_timeout);

		if (dist_bottom_valid) {
			/* surface distance prediction */
			surface_offset += surface_offset_rate * dt;

			/* surface distance correction */
			if (sonar_valid) {
				surface_offset_rate -= corr_sonar * 0.5f * position_estimator_inav_parameters.w_z_sonar * position_estimator_inav_parameters.w_z_sonar * dt;
				surface_offset -= corr_sonar * position_estimator_inav_parameters.w_z_sonar * dt;
			}
		}

		float w_xy_gps_p = position_estimator_inav_parameters.w_xy_gps_p * w_gps_xy;
		float w_xy_gps_v = position_estimator_inav_parameters.w_xy_gps_v * w_gps_xy;
		float w_z_gps_p = position_estimator_inav_parameters.w_z_gps_p * w_gps_z;

		/* reduce GPS weight if optical flow is good */
		if (use_flow && flow_accurate) {
			w_xy_gps_p *= position_estimator_inav_parameters.w_gps_flow;
			w_xy_gps_v *= position_estimator_inav_parameters.w_gps_flow;
		}

		/* baro offset correction */
		if (use_gps_z) {
			float offs_corr = corr_gps[2][0] * w_z_gps_p * dt;
			baro_offset += offs_corr;
			corr_baro += offs_corr;
		}

		/* accelerometer bias correction */
		float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };

		if (use_gps_xy) {
			accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;
			accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
		}

		if (use_gps_z) {
			accel_bias_corr[2] -= corr_gps[2][0] * w_z_gps_p * w_z_gps_p;
		}

		if (use_flow) {
			accel_bias_corr[0] -= corr_flow[0] * position_estimator_inav_parameters.w_xy_flow;
			accel_bias_corr[1] -= corr_flow[1] * position_estimator_inav_parameters.w_xy_flow;
		}

		accel_bias_corr[2] -= corr_baro * position_estimator_inav_parameters.w_z_baro * position_estimator_inav_parameters.w_z_baro;

		/* transform error vector from NED frame to body frame */
		for (i = 0; i < 3; i++) {
			float c = 0.0f;

			for (j = 0; j < 3; j++) {
				c += att.R[j][i] * accel_bias_corr[j];
			}

			acc_bias[i] += c * position_estimator_inav_parameters.w_acc_bias * dt;
		}

		/* inertial filter prediction for altitude */
		inertial_filter_predict(dt, z_est);

		/* inertial filter correction for altitude */
		inertial_filter_correct(corr_baro, dt, z_est, 0, position_estimator_inav_parameters.w_z_baro);
		inertial_filter_correct(corr_gps[2][0], dt, z_est, 0, w_z_gps_p);
		inertial_filter_correct(corr_acc[2], dt, z_est, 2, position_estimator_inav_parameters.w_z_acc);

		float x_est_prev[3], y_est_prev[3];

		memcpy(x_est_prev, x_est, sizeof(x_est));
		memcpy(y_est_prev, y_est, sizeof(y_est));

		if (can_estimate_xy) {
			/* inertial filter prediction for position */
			inertial_filter_predict(dt, x_est);
			inertial_filter_predict(dt, y_est);

			if (!check_finite(x_est[0]) || !check_finite(y_est[0])) {
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
			}

			/* inertial filter correction for position */
			inertial_filter_correct(corr_acc[0], dt, x_est, 2, position_estimator_inav_parameters.w_xy_acc);
			inertial_filter_correct(corr_acc[1], dt, y_est, 2, position_estimator_inav_parameters.w_xy_acc);

			if (use_flow) {
				inertial_filter_correct(corr_flow[0], dt, x_est, 1, position_estimator_inav_parameters.w_xy_flow * w_flow);
				inertial_filter_correct(corr_flow[1], dt, y_est, 1, position_estimator_inav_parameters.w_xy_flow * w_flow);
			}

			if (use_gps_xy) {
				inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
				inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

				if (gps.vel_ned_valid && t < gps.timestamp_velocity + gps_topic_timeout) {
					inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
					inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);
				}
			}

			if (!check_finite(x_est[0]) || !check_finite(y_est[0])) {
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
				memset(corr_acc, 0, sizeof(corr_acc));
				memset(corr_gps, 0, sizeof(corr_gps));
				memset(corr_flow, 0, sizeof(corr_flow));
			}
		}

		/* detect land */
		alt_avg += (- z_est[0] - alt_avg) * dt / position_estimator_inav_parameters.land_t;
		float alt_disp2 = - z_est[0] - alt_avg;
		alt_disp2 = alt_disp2 * alt_disp2;
		float land_disp2 = position_estimator_inav_parameters.land_disp * position_estimator_inav_parameters.land_disp;
		/* get actual thrust output */
		float thrust = armed.armed ? actuator.control[3] : 0.0f;

		if (landed) {
			if (alt_disp2 > land_disp2 && thrust > position_estimator_inav_parameters.land_thr) {
				landed = 0;
				landed_time = 0;
			}

		} else {
			if (alt_disp2 < land_disp2 && thrust < position_estimator_inav_parameters.land_thr) {
				if (landed_time == 0) {
					landed_time = t;    // land detected first time

				} else {
					if (t > landed_time + position_estimator_inav_parameters.land_t * 1000000.0f) {
						landed = 1;
						landed_time = 0;
					}
				}

			} else {
				landed_time = 0;
			}
		}

		if (t > pub_last + pub_interval) {
			pub_last = t;
			/* publish local position */
			local_pos.xy_valid = can_estimate_xy && use_gps_xy;
			local_pos.v_xy_valid = can_estimate_xy;
			local_pos.xy_global = local_pos.xy_valid && use_gps_xy;
			local_pos.z_global = local_pos.z_valid && use_gps_z;
			local_pos.x = x_est[0];
			local_pos.vx = x_est[1];
			local_pos.y = y_est[0];
			local_pos.vy = y_est[1];
			local_pos.z = z_est[0];
			local_pos.vz = z_est[1];
			local_pos.landed = landed;
			local_pos.yaw = att.yaw;
			local_pos.dist_bottom_valid = dist_bottom_valid;

			if (dist_bottom_valid) {
				local_pos.dist_bottom = -z_est[0] - surface_offset;
				local_pos.dist_bottom_rate = -z_est[1] - surface_offset_rate;
			}

			local_pos.timestamp = t;
			orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);

			
			/* publish global position */
			global_pos.valid = local_pos.xy_global;

			if (local_pos.xy_global) {
				double est_lat, est_lon;
				map_projection_reproject(local_pos.x, local_pos.y, &est_lat, &est_lon);
				global_pos.latitude = est_lat*1e7;
				global_pos.longitude = est_lon*1e7;
				global_pos.time_gps_usec = gps.time_gps_usec;
			}

			/* set valid values even if position is not valid */
			if (local_pos.v_xy_valid) {
				global_pos.vx = local_pos.vx;
				global_pos.vy = local_pos.vy;
			}

			if (local_pos.z_global) {
				global_pos.altitude = local_pos.ref_alt - local_pos.z;
				global_pos.ground_level = local_pos.ref_alt;
			}

			if (local_pos.z_valid) {
				global_pos.relative_altitude = - local_pos.z;
			}

			if (local_pos.v_z_valid) {
				global_pos.vz = local_pos.vz;
			}

			global_pos.yaw = local_pos.yaw;
			global_pos.landed = landed;
			orb_publish(ORB_ID(vehicle_global_position), vehicle_global_position_pub, &global_pos);
		}
	}


	/*
	 * do unsubscriptions
	 */
	orb_unsubscribe(ORB_ID(parameter_update), parameter_update_sub, pthread_self());
	orb_unsubscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, pthread_self());
	orb_unsubscribe(ORB_ID(actuator_armed), armed_sub, pthread_self());
	orb_unsubscribe(ORB_ID(sensor_combined), sensor_combined_sub, pthread_self());
	orb_unsubscribe(ORB_ID(sensor_optical_flow), optical_flow_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_attitude), vehicle_attitude_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, pthread_self());

	/*
	 * do unadvertises
	 */
	orb_unadvertise(ORB_ID(vehicle_local_position), vehicle_local_position_pub, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_global_position), vehicle_global_position_pub, pthread_self());

	return 0;
}
