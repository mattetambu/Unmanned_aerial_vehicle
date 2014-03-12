/*
 *  @file    ECL_tecs.h
 *  @brief   Combined Total Energy Speed & Height Control.
 *
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 */

#ifndef ECL_TECS_H
#define ECL_TECS_H

	#include <stdint.h>
	#include "../../uav_library/common.h"
	#include "../../uav_library/math/Dcm.h"
	#include "../../uav_library/math/Vector3f.h"


	void ECL_tecs_init ();
	bool_t ECL_tecs_airspeed_sensor_enabled ();
	void ECL_tecs_enable_airspeed (bool_t enabled);

	// Update of the estimated height and height rate internal state
	// Update of the inertial speed rate internal state
	// Should be called at 50Hz or greater
	int ECL_tecs_update_50hz (float baro_altitude, float airspeed, Dcm *rotMat, Vector3f *accel_body, Vector3f *accel_earth);

	// Update the control loop calculations
	void ECL_tecs_update_pitch_throttle (Dcm *rotMat, float pitch, float baro_altitude, float hgt_dem, float EAS_dem, float indicated_airspeed, float EAS2TAS, bool_t climbOutDem, float ptchMinCO,
				   float throttle_min, float throttle_max, float throttle_cruise,
				   float pitch_limit_min, float pitch_limit_max);

	float ECL_tecs_get_throttle_demand (void);
	int32_t ECL_tecs_get_throttle_demand_percent (void);
	float ECL_tecs_get_pitch_demand ();
	int32_t ECL_tecs_get_pitch_demand_cd ();
	float ECL_tecs_get_VXdot (void);
	float ECL_tecs_get_speed_weight ();

	void ECL_tecs_set_time_const (float time_const);
	void ECL_tecs_set_min_sink_rate (float rate);
	void ECL_tecs_set_max_sink_rate (float sink_rate);
	void ECL_tecs_set_max_climb_rate (float climb_rate);
	void ECL_tecs_set_throttle_damp (float throttle_damp);
	void ECL_tecs_set_integrator_gain (float gain);
	void ECL_tecs_set_vertical_accel_limit (float limit);
	void ECL_tecs_set_height_comp_filter_omega (float omega);
	void ECL_tecs_set_speed_comp_filter_omega (float omega);
	void ECL_tecs_set_roll_throttle_compensation (float compensation);
	void ECL_tecs_set_speed_weight (float weight);
	void ECL_tecs_set_pitch_damping (float damping);
	void ECL_tecs_set_throttle_slewrate (float slewrate);
	void ECL_tecs_set_indicated_airspeed_min (float airspeed);
	void ECL_tecs_set_indicated_airspeed_max (float airspeed);

#endif //TECS_H
