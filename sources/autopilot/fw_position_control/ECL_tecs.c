/**
 * @file ECL_tecs.c
 *
 * @author Paul Riseborough
 *
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle and switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, integrator and damping gains and the use
 *    of easy to measure aircraft performance data
 *
 */

#include "ECL_tecs.h"
#include "../../uav_library/common.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/math/limits.h"
#include "../../uav_library/math/Dcm.h"
#include "../../uav_library/math/Vector3f.h"


struct ECL_tecs {
	// Last time update_50Hz was called
	absolute_time _update_50hz_last_usec;

	// Last time update_speed was called
	absolute_time _update_speed_last_usec;

	// Last time update_pitch_throttle was called
	absolute_time _update_pitch_throttle_last_usec;

	// TECS tuning parameters
	float _hgtCompFiltOmega;
	float _spdCompFiltOmega;
	float _maxClimbRate;
	float _minSinkRate;
	float _maxSinkRate;
	float _timeConst;
	float _ptchDamp;
	float _thrDamp;
	float _integGain;
	float _vertAccLim;
	float _rollComp;
	float _spdWeight;

	// throttle demand in the range from 0.0 to 1.0
	float _throttle_dem;

	// pitch angle demand in radians
	float _pitch_dem;

	// Integrator state 1 - height filter second derivative
	float _integ1_state;

	// Integrator state 2 - height rate
	float _integ2_state;

	// Integrator state 3 - height
	float _integ3_state;

	// Integrator state 4 - airspeed filter first derivative
	float _integ4_state;

	// Integrator state 5 - true airspeed
	float _integ5_state;

	// Integrator state 6 - throttle integrator
	float _integ6_state;

	// Integrator state 7 - pitch integrator
	float _integ7_state;

	// throttle demand rate limiter state
	float _last_throttle_dem;

	// pitch demand rate limiter state
	float _last_pitch_dem;

	// Rate of change of speed along X axis
	float _vel_dot;

	// Equivalent airspeed
	float _EAS;

	// True airspeed limits
	float _TASmax;
	float _TASmin;

	// Current and last true airspeed demand
	float _TAS_dem;
	float _TAS_dem_last;

	// Equivalent airspeed demand
	float _EAS_dem;

	// height demands
	float _hgt_dem;
	float _hgt_dem_in_old;
	float _hgt_dem_adj;
	float _hgt_dem_adj_last;
	float _hgt_rate_dem;
	float _hgt_dem_prev;

	// Speed demand after application of rate limiting
	// This is the demand tracked by the TECS control loops
	float _TAS_dem_adj;

	// Speed rate demand after application of rate limiting
	// This is the demand tracked by the TECS control loops
	float _TAS_rate_dem;

	// Total energy rate filter state
	float _STEdotErrLast;

	// Underspeed condition
	bool_t _underspeed;

	// Bad descent condition caused by unachievable airspeed demand
	bool_t _badDescent;

	// climbout mode
	bool_t _climbOutDem;

	// throttle demand before limiting
	float _throttle_dem_unc;

	// pitch demand before limiting
	float _pitch_dem_unc;

	// Maximum and minimum specific total energy rate limits
	float _STEdot_max;
	float _STEdot_min;

	// Maximum and minimum floating point throttle limits
	float _THRmaxf;
	float _THRminf;

	// Maximum and minimum floating point pitch limits
	float _PITCHmaxf;
	float _PITCHminf;

	// Specific energy quantities
	float _SPE_dem;
	float _SKE_dem;
	float _SPEdot_dem;
	float _SKEdot_dem;
	float _SPE_est;
	float _SKE_est;
	float _SPEdot;
	float _SKEdot;

	// Specific energy error quantities
	float _STE_error;

	// Time since last update of main TECS loop (seconds)
	float _DT;

	bool_t _airspeed_enabled;
	float _throttle_slewrate;
	float _indicated_airspeed_min;
	float _indicated_airspeed_max;
} et;



void ECL_tecs_init ()
{
	et._airspeed_enabled = 0;
	et._throttle_slewrate = 0.0f;
	et._climbOutDem = 0;
	et._hgt_dem_prev = 0.0f;
	et._hgt_dem_adj_last = 0.0f;
	et._hgt_dem_in_old = 0.0f;
	et._TAS_dem_last = 0.0f;
	et._TAS_dem_adj = 0.0f;
	et._TAS_dem = 0.0f;

	et._integ1_state = 0.0f;
	et._integ2_state = 0.0f;
	et._integ3_state = 0.0f;
	et._integ4_state = 0.0f;
	et._integ5_state = 0.0f;
	et._integ6_state = 0.0f;
	et._integ7_state = 0.0f;

	et._pitch_dem = 0.0f;
	et._last_pitch_dem = 0.0f;

	et._SPE_dem = 0.0f;
	et._SKE_dem = 0.0f;
	et._SPEdot_dem = 0.0f;
	et._SKEdot_dem = 0.0f;
	et._SPE_est = 0.0f;
	et._SKE_est = 0.0f;
	et._SPEdot = 0.0f;
	et._SKEdot = 0.0f;
}


bool_t ECL_tecs_airspeed_sensor_enabled () {
	return et._airspeed_enabled;
}

void ECL_tecs_enable_airspeed (bool_t enabled) {
	et._airspeed_enabled = enabled;
}

// demanded throttle in percentage
// should return 0 to 100
float ECL_tecs_get_throttle_demand (void) {
	return et._throttle_dem;
}
int32_t ECL_tecs_get_throttle_demand_percent (void) {
	return ECL_tecs_get_throttle_demand ();
}


float ECL_tecs_get_pitch_demand () {
	return et._pitch_dem;
}

// demanded pitch angle in centi-degrees
// should return between -9000 to +9000
int32_t ECL_tecs_get_pitch_demand_cd () {
	return (int32_t) (ECL_tecs_get_pitch_demand () * 5729.5781f);
}

// Rate of change of velocity along X body axis in m/s^2
float ECL_tecs_get_VXdot (void) {
	return et._vel_dot;
}


float ECL_tecs_get_speed_weight () {
	return et._spdWeight;
}

void ECL_tecs_set_time_const (float time_const) {
	et._timeConst = time_const;
}

void ECL_tecs_set_min_sink_rate (float rate) {
	et._minSinkRate = rate;
}

void ECL_tecs_set_max_sink_rate (float sink_rate) {
	et._maxSinkRate = sink_rate;
}

void ECL_tecs_set_max_climb_rate (float climb_rate) {
	et._maxClimbRate = climb_rate;
}

void ECL_tecs_set_throttle_damp (float throttle_damp) {
	et._thrDamp = throttle_damp;
}

void ECL_tecs_set_integrator_gain (float gain) {
	et._integGain = gain;
}

void ECL_tecs_set_vertical_accel_limit (float limit) {
	et._vertAccLim = limit;
}

void ECL_tecs_set_height_comp_filter_omega (float omega) {
	et._hgtCompFiltOmega = omega;
}

void ECL_tecs_set_speed_comp_filter_omega (float omega) {
	et._spdCompFiltOmega = omega;
}

void ECL_tecs_set_roll_throttle_compensation (float compensation) {
	et._rollComp = compensation;
}

void ECL_tecs_set_speed_weight (float weight) {
	et._spdWeight = weight;
}

void ECL_tecs_set_pitch_damping (float damping) {
	et._ptchDamp = damping;
}

void ECL_tecs_set_throttle_slewrate (float slewrate) {
	et._throttle_slewrate = slewrate;
}

void ECL_tecs_set_indicated_airspeed_min (float airspeed) {
	et._indicated_airspeed_min = airspeed;
}

void ECL_tecs_set_indicated_airspeed_max (float airspeed) {
	et._indicated_airspeed_max = airspeed;
}


int ECL_tecs_update_50hz(float baro_altitude, float airspeed, Dcm *rotMat, Vector3f *accel_body, Vector3f *accel_earth)
{
	// Implement third order complementary filter for height and height rate
	// estimted height rate = et._integ2_state
	// estimated height     = et._integ3_state
	// Reference Paper :
	// Optimising the Gains of the Baro-Inertial Vertical Channel
	// Widnall W.S, Sinha P.K,
	// AIAA Journal of Guidance and Control, 78-1307R

	// Calculate time in seconds since last update
	absolute_time now = get_absolute_time();
	float DT = max((uint64_t) (now - et._update_50hz_last_usec), 0ULL) * 1.0e-6f;
	float rotMat_20, accel_earth2, accel_body0;
	MATHLIB_ASSERT (Vector3f_getZ (accel_earth, &accel_earth2));

	// printf("dt: %10.6f baro alt: %6.2f eas: %6.2f R(0,0): %6.2f, R(1,1): %6.2f\naccel body: %6.2f %6.2f %6.2f\naccel earth: %6.2f %6.2f %6.2f\n",
	// 	DT, baro_altitude, airspeed, rotMat(0, 0), rotMat(1, 1), accel_body(0), accel_body(1), accel_body(2),
	// 	accel_earth(0), accel_earth(1), accel_earth(2));

	if (DT > 1.0f) {
		et._integ3_state = baro_altitude;
		et._integ2_state = 0.0f;
		et._integ1_state = 0.0f;
		DT            = 0.02f; // when first starting TECS, use a
		// small time constant
	}

	et._update_50hz_last_usec = now;
	et._EAS = airspeed;

	// Get height acceleration
	float hgt_ddot_mea = -(accel_earth2 + CONSTANTS_ONE_G);
	// Perform filter calculation using backwards Euler integration
	// Coefficients selected to place all three filter poles at omega
	float omega2 = et._hgtCompFiltOmega * et._hgtCompFiltOmega;
	float hgt_err = baro_altitude - et._integ3_state;
	float integ1_input = hgt_err * omega2 * et._hgtCompFiltOmega;
	et._integ1_state = et._integ1_state + integ1_input * DT;
	float integ2_input = et._integ1_state + hgt_ddot_mea + hgt_err * omega2 * 3.0f;
	et._integ2_state = et._integ2_state + integ2_input * DT;
	float integ3_input = et._integ2_state + hgt_err * et._hgtCompFiltOmega * 3.0f;

	// If more than 1 second has elapsed since last update then reset the integrator state
	// to the measured height
	if (DT > 1.0f) {
		et._integ3_state = baro_altitude;

	} else {
		et._integ3_state = et._integ3_state + integ3_input * DT;
	}

	// Update and average speed rate of change
	// Only required if airspeed is being measured and controlled
	float temp = 0;

	if (!check_out_of_bounds (airspeed, et._indicated_airspeed_min, et._indicated_airspeed_max) && ECL_tecs_airspeed_sensor_enabled()) {
		// Get DCM
		// Calculate speed rate of change
		// XXX check
		MATHLIB_ASSERT (Dcm_get_data (rotMat, &rotMat_20, 2, 0));
		MATHLIB_ASSERT (Vector3f_getX (accel_body, &accel_body0));
		temp = rotMat_20 * CONSTANTS_ONE_G + accel_body0;
		// take 5 point moving average
		//et._vel_dot = _vdot_filter.apply(temp);
		// XXX resolve this properly
		et._vel_dot = 0.9f * et._vel_dot + 0.1f * temp;

	} else {
		et._vel_dot = 0.0f;
	}

	return 0;
}

void ECL_tecs_update_speed(float airspeed_demand, float indicated_airspeed, float indicated_airspeed_min, float indicated_airspeed_max, float EAS2TAS)
{
	// Calculate time in seconds since last update
	absolute_time now = get_absolute_time();
	float DT = max((uint64_t) (now - et._update_speed_last_usec), 0ULL) * 1.0e-6f;
	et._update_speed_last_usec = now;

	// Convert equivalent airspeeds to true airspeeds

	et._EAS_dem = airspeed_demand;
	et._TAS_dem  = et._EAS_dem * EAS2TAS;
	et._TASmax   = indicated_airspeed_max * EAS2TAS;
	et._TASmin   = indicated_airspeed_min * EAS2TAS;

	// Reset states of time since last update is too large
	if (DT > 1.0f) {
		et._integ5_state = (et._EAS * EAS2TAS);
		et._integ4_state = 0.0f;
		DT            = 0.1f; // when first starting TECS, use a
		// small time constant
	}

	// Get airspeed or default to halfway between min and max if
	// airspeed is not being used and set speed rate to zero
	if (check_out_of_bounds (indicated_airspeed, et._indicated_airspeed_min, et._indicated_airspeed_max) || !ECL_tecs_airspeed_sensor_enabled()) {
		// If no airspeed available use average of min and max
		et._EAS = 0.5f * (indicated_airspeed_min + indicated_airspeed_max);

	} else {
		et._EAS = indicated_airspeed;
	}

	// Implement a second order complementary filter to obtain a
	// smoothed airspeed estimate
	// airspeed estimate is held in et._integ5_state
	float aspdErr = (et._EAS * EAS2TAS) - et._integ5_state;
	float integ4_input = aspdErr * et._spdCompFiltOmega * et._spdCompFiltOmega;

	// Prevent state from winding up
	if (et._integ5_state < 3.1f) {
		integ4_input = max(integ4_input , 0.0f);
	}

	et._integ4_state = et._integ4_state + integ4_input * DT;
	float integ5_input = et._integ4_state + et._vel_dot + aspdErr * et._spdCompFiltOmega * 1.4142f;
	et._integ5_state = et._integ5_state + integ5_input * DT;
	// limit the airspeed to a minimum of 3 m/s
	et._integ5_state = max(et._integ5_state, 3.0f);

}

void ECL_tecs_update_speed_demand(void)
{
	// Set the airspeed demand to the minimum value if an underspeed condition exists
	// or a bad descent condition exists
	// This will minimise the rate of descent resulting from an engine failure,
	// enable the maximum climb rate to be achieved and prevent continued full power descent
	// into the ground due to an unachievable airspeed value
	if ((et._badDescent) || (et._underspeed)) {
		et._TAS_dem     = et._TASmin;
	}

	// Constrain speed demand
	et._TAS_dem = constrain(et._TAS_dem, et._TASmin, et._TASmax);

	// calculate velocity rate limits based on physical performance limits
	// provision to use a different rate limit if bad descent or underspeed condition exists
	// Use 50% of maximum energy rate to allow margin for total energy contgroller
	float velRateMax;
	float velRateMin;

	if ((et._badDescent) || (et._underspeed)) {
		velRateMax = 0.5f * et._STEdot_max / et._integ5_state;
		velRateMin = 0.5f * et._STEdot_min / et._integ5_state;

	} else {
		velRateMax = 0.5f * et._STEdot_max / et._integ5_state;
		velRateMin = 0.5f * et._STEdot_min / et._integ5_state;
	}

	// Apply rate limit
	if ((et._TAS_dem - et._TAS_dem_adj) > (velRateMax * 0.1f)) {
		et._TAS_dem_adj = et._TAS_dem_adj + velRateMax * 0.1f;
		et._TAS_rate_dem = velRateMax;

	} else if ((et._TAS_dem - et._TAS_dem_adj) < (velRateMin * 0.1f)) {
		et._TAS_dem_adj = et._TAS_dem_adj + velRateMin * 0.1f;
		et._TAS_rate_dem = velRateMin;

	} else {
		et._TAS_dem_adj = et._TAS_dem;
		et._TAS_rate_dem = (et._TAS_dem - et._TAS_dem_last) / 0.1f;
	}

	// Constrain speed demand again to protect against bad values on initialisation.
	et._TAS_dem_adj = constrain(et._TAS_dem_adj, et._TASmin, et._TASmax);
	et._TAS_dem_last = et._TAS_dem;
}

void ECL_tecs_update_height_demand(float demand)
{
	// Apply 2 point moving average to demanded height
	// This is required because height demand is only updated at 5Hz
	et._hgt_dem = 0.5f * (demand + et._hgt_dem_in_old);
	et._hgt_dem_in_old = et._hgt_dem;

	// printf("hgt_dem: %6.2f hgt_dem_last: %6.2f max_climb_rate: %6.2f\n", et._hgt_dem, et._hgt_dem_prev,
	// 	et._maxClimbRate);

	// Limit height rate of change
	if ((et._hgt_dem - et._hgt_dem_prev) > (et._maxClimbRate * 0.1f)) {
		et._hgt_dem = et._hgt_dem_prev + et._maxClimbRate * 0.1f;

	} else if ((et._hgt_dem - et._hgt_dem_prev) < (-et._maxSinkRate * 0.1f)) {
		et._hgt_dem = et._hgt_dem_prev - et._maxSinkRate * 0.1f;
	}

	et._hgt_dem_prev = et._hgt_dem;

	// Apply first order lag to height demand
	et._hgt_dem_adj = 0.05f * et._hgt_dem + 0.95f * et._hgt_dem_adj_last;
	et._hgt_rate_dem = (et._hgt_dem_adj - et._hgt_dem_adj_last) / 0.1f;
	et._hgt_dem_adj_last = et._hgt_dem_adj;

	// printf("hgt_dem: %6.2f hgt_dem_adj: %6.2f hgt_dem_last: %6.2f hgt_rate_dem: %6.2f\n", et._hgt_dem, et._hgt_dem_adj, et._hgt_dem_adj_last,
	// 	et._hgt_rate_dem);
}

void ECL_tecs_detect_underspeed(void)
{
	if (((et._integ5_state < et._TASmin * 0.9f) && (et._throttle_dem >= et._THRmaxf * 0.95f)) || ((et._integ3_state < et._hgt_dem_adj) && et._underspeed)) {
		et._underspeed = 1 /* true */;

	} else {
		et._underspeed = 0 /* false */;
	}
}

void ECL_tecs_update_energies(void)
{
	// Calculate specific energy demands
	et._SPE_dem = et._hgt_dem_adj * CONSTANTS_ONE_G;
	et._SKE_dem = 0.5f * et._TAS_dem_adj * et._TAS_dem_adj;

	// Calculate specific energy rate demands
	et._SPEdot_dem = et._hgt_rate_dem * CONSTANTS_ONE_G;
	et._SKEdot_dem = et._integ5_state * et._TAS_rate_dem;

	// Calculate specific energy
	et._SPE_est = et._integ3_state * CONSTANTS_ONE_G;
	et._SKE_est = 0.5f * et._integ5_state * et._integ5_state;

	// Calculate specific energy rate
	et._SPEdot = et._integ2_state * CONSTANTS_ONE_G;
	et._SKEdot = et._integ5_state * et._vel_dot;
}

int ECL_tecs_update_throttle(float throttle_cruise, Dcm *rotMat)
{
	// Calculate total energy values
	et._STE_error = et._SPE_dem - et._SPE_est + et._SKE_dem - et._SKE_est;
	float STEdot_dem = constrain((et._SPEdot_dem + et._SKEdot_dem), et._STEdot_min, et._STEdot_max);
	float STEdot_error = STEdot_dem - et._SPEdot - et._SKEdot;
	float rotMat_01, rotMat_11;

	// Apply 0.5 second first order filter to STEdot_error
	// This is required to remove accelerometer noise from the  measurement
	STEdot_error = 0.2f * STEdot_error + 0.8f * et._STEdotErrLast;
	et._STEdotErrLast = STEdot_error;

	// Calculate throttle demand
	// If underspeed condition is set, then demand full throttle
	if (et._underspeed) {
		et._throttle_dem_unc = 1.0f;

	} else {
		// Calculate gain scaler from specific energy error to throttle
		float K_STE2Thr = 1 / (et._timeConst * (et._STEdot_max - et._STEdot_min));

		// Calculate feed-forward throttle
		float ff_throttle = 0;
		float nomThr = throttle_cruise;
		// Use the demanded rate of change of total energy as the feed-forward demand, but add
		// additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
		// drag increase during turns.

		MATHLIB_ASSERT (Dcm_get_data (rotMat, &rotMat_01, 0, 1));
		MATHLIB_ASSERT (Dcm_get_data (rotMat, &rotMat_11, 1, 1));
		float cosPhi = sqrtf((rotMat_01 * rotMat_01) + (rotMat_11 * rotMat_11));
		STEdot_dem = STEdot_dem + et._rollComp * (1.0f / constrain(cosPhi * cosPhi , 0.1f, 1.0f) - 1.0f);

		if (STEdot_dem >= 0) {
			ff_throttle = nomThr + STEdot_dem / et._STEdot_max * (1.0f - nomThr);

		} else {
			ff_throttle = nomThr - STEdot_dem / et._STEdot_min * nomThr;
		}

		// Calculate PD + FF throttle
		et._throttle_dem = (et._STE_error + STEdot_error * et._thrDamp) * K_STE2Thr + ff_throttle;

		// Rate limit PD + FF throttle
		// Calculate the throttle increment from the specified slew time
		if (fabsf(et._throttle_slewrate) < 0.01f) {
			float thrRateIncr = et._DT * (et._THRmaxf - et._THRminf) * et._throttle_slewrate;

			et._throttle_dem = constrain(et._throttle_dem,
						  et._last_throttle_dem - thrRateIncr,
						  et._last_throttle_dem + thrRateIncr);
			et._last_throttle_dem = et._throttle_dem;
		}


		// Calculate integrator state upper and lower limits
		// Set to a value thqat will allow 0.1 (10%) throttle saturation to allow for noise on the demand
		float integ_max = (et._THRmaxf - et._throttle_dem + 0.1f);
		float integ_min = (et._THRminf - et._throttle_dem - 0.1f);

		// Calculate integrator state, constraining state
		// Set integrator to a max throttle value dduring climbout
		et._integ6_state = et._integ6_state + (et._STE_error * et._integGain) * et._DT * K_STE2Thr;

		if (et._climbOutDem) {
			et._integ6_state = integ_max;

		} else {
			et._integ6_state = constrain(et._integ6_state, integ_min, integ_max);
		}

		// Sum the components.
		// Only use feed-forward component if airspeed is not being used
		if (ECL_tecs_airspeed_sensor_enabled()) {
			et._throttle_dem = et._throttle_dem + et._integ6_state;

		} else {
			et._throttle_dem = ff_throttle;
		}
	}

	// Constrain throttle demand
	et._throttle_dem = constrain(et._throttle_dem, et._THRminf, et._THRmaxf);

	return 0;
}

void ECL_tecs_detect_bad_descent(void)
{
	// Detect a demanded airspeed too high for the aircraft to achieve. This will be
	// evident by the the following conditions:
	// 1) Underspeed protection not active
	// 2) Specific total energy error > 200 (greater than ~20m height error)
	// 3) Specific total energy reducing
	// 4) throttle demand > 90%
	// If these four conditions exist simultaneously, then the protection
	// mode will be activated.
	// Once active, the following condition are required to stay in the mode
	// 1) Underspeed protection not active
	// 2) Specific total energy error > 0
	// This mode will produce an undulating speed and height response as it cuts in and out but will prevent the aircraft from descending into the ground if an unachievable speed demand is set
	float STEdot = et._SPEdot + et._SKEdot;

	if ((!et._underspeed && (et._STE_error > 200.0f) && (STEdot < 0.0f) && (et._throttle_dem >= et._THRmaxf * 0.9f)) || (et._badDescent && !et._underspeed && (et._STE_error > 0.0f))) {
		et._badDescent = 1 /* true */;

	} else {
		et._badDescent = 0 /* false */;
	}
}

void ECL_tecs_update_pitch(void)
{
	// Calculate Speed/Height Control Weighting
	// This is used to determine how the pitch control prioritises speed and height control
	// A weighting of 1 provides equal priority (this is the normal mode of operation)
	// A SKE_weighting of 0 provides 100% priority to height control. This is used when no airspeed measurement is available
	// A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected
	// or during takeoff/climbout where a minimum pitch angle is set to ensure height is gained. In this instance, if airspeed
	// rises above the demanded value, the pitch angle will be increased by the TECS controller.
	float SKE_weighting = constrain(et._spdWeight, 0.0f, 2.0f);

	if ((et._underspeed || et._climbOutDem) && ECL_tecs_airspeed_sensor_enabled()) {
		SKE_weighting = 2.0f;

	} else if (!ECL_tecs_airspeed_sensor_enabled()) {
		SKE_weighting = 0.0f;
	}

	float SPE_weighting = 2.0f - SKE_weighting;

	// Calculate Specific Energy Balance demand, and error
	float SEB_dem      = et._SPE_dem * SPE_weighting - et._SKE_dem * SKE_weighting;
	float SEBdot_dem   = et._SPEdot_dem * SPE_weighting - et._SKEdot_dem * SKE_weighting;
	float SEB_error    = SEB_dem - (et._SPE_est * SPE_weighting - et._SKE_est * SKE_weighting);
	float SEBdot_error = SEBdot_dem - (et._SPEdot * SPE_weighting - et._SKEdot * SKE_weighting);

	// Calculate integrator state, constraining input if pitch limits are exceeded
	float integ7_input = SEB_error * et._integGain;

	if (et._pitch_dem_unc > et._PITCHmaxf) {
		integ7_input = min(integ7_input, et._PITCHmaxf - et._pitch_dem_unc);

	} else if (et._pitch_dem_unc < et._PITCHminf) {
		integ7_input = max(integ7_input, et._PITCHminf - et._pitch_dem_unc);
	}

	et._integ7_state = et._integ7_state + integ7_input * et._DT;

	// Apply max and min values for integrator state that will allow for no more than
	// 5deg of saturation. This allows for some pitch variation due to gusts before the
	// integrator is clipped. Otherwise the effectiveness of the integrator will be reduced in turbulence
	float gainInv = (et._integ5_state * et._timeConst * CONSTANTS_ONE_G);
	float temp = SEB_error + SEBdot_error * et._ptchDamp + SEBdot_dem * et._timeConst;
	et._integ7_state = constrain(et._integ7_state, (gainInv * (et._PITCHminf - 0.0783f)) - temp, (gainInv * (et._PITCHmaxf + 0.0783f)) - temp);

	// Calculate pitch demand from specific energy balance signals
	et._pitch_dem_unc = (temp + et._integ7_state) / gainInv;

	// Constrain pitch demand
	et._pitch_dem = constrain(et._pitch_dem_unc, et._PITCHminf, et._PITCHmaxf);

	// Rate limit the pitch demand to comply with specified vertical
	// acceleration limit
	float ptchRateIncr = et._DT * et._vertAccLim / et._integ5_state;

	if ((et._pitch_dem - et._last_pitch_dem) > ptchRateIncr) {
		et._pitch_dem = et._last_pitch_dem + ptchRateIncr;

	} else if ((et._pitch_dem - et._last_pitch_dem) < -ptchRateIncr) {
		et._pitch_dem = et._last_pitch_dem - ptchRateIncr;
	}

	et._last_pitch_dem = et._pitch_dem;
}

void ECL_tecs_initialise_states(float pitch, float throttle_cruise, float baro_altitude, float ptchMinCO_rad)
{
	// Initialise states and variables if DT > 1 second or in climbout
	if (et._DT > 1.0f) {
		et._integ6_state      = 0.0f;
		et._integ7_state      = 0.0f;
		et._last_throttle_dem = throttle_cruise;
		et._last_pitch_dem    = pitch;
		et._hgt_dem_adj_last  = baro_altitude;
		et._hgt_dem_adj       = et._hgt_dem_adj_last;
		et._hgt_dem_prev      = et._hgt_dem_adj_last;
		et._hgt_dem_in_old    = et._hgt_dem_adj_last;
		et._TAS_dem_last      = et._TAS_dem;
		et._TAS_dem_adj       = et._TAS_dem;
		et._underspeed        = 0 /* false */;
		et._badDescent        = 0 /* false */;
		et._DT                = 0.1f; // when first starting TECS, use a
		// small time constant

	} else if (et._climbOutDem) {
		et._PITCHminf         = ptchMinCO_rad;
		et._THRminf           = et._THRmaxf - 0.01f;
		et._hgt_dem_adj_last  = baro_altitude;
		et._hgt_dem_adj       = et._hgt_dem_adj_last;
		et._hgt_dem_prev      = et._hgt_dem_adj_last;
		et._TAS_dem_last      = et._TAS_dem;
		et._TAS_dem_adj       = et._TAS_dem;
		et._underspeed        = 0 /* false */;
		et._badDescent 	   = 0 /* false */;
	}
}

void ECL_tecs_update_STE_rate_lim(void)
{
	// Calculate Specific Total Energy Rate Limits
	// This is a tivial calculation at the moment but will get bigger once we start adding altitude effects
	et._STEdot_max = et._maxClimbRate * CONSTANTS_ONE_G;
	et._STEdot_min = - et._minSinkRate * CONSTANTS_ONE_G;
}

void ECL_tecs_update_pitch_throttle(Dcm *rotMat, float pitch, float baro_altitude, float hgt_dem, float EAS_dem, float indicated_airspeed, float EAS2TAS, bool_t climbOutDem, float ptchMinCO,
				 float throttle_min, float throttle_max, float throttle_cruise,
				 float pitch_limit_min, float pitch_limit_max)
{
	// Calculate time in seconds since last update
	absolute_time now = get_absolute_time();
	et._DT = max((uint64_t) (now - et._update_pitch_throttle_last_usec), 0ULL) * 1.0e-6f;
	et._update_pitch_throttle_last_usec = now;

	// printf("tecs in: dt:%10.6f pitch: %6.2f baro_alt: %6.2f alt sp: %6.2f\neas sp: %6.2f eas: %6.2f, eas2tas: %6.2f\n %s pitch min C0: %6.2f thr min: %6.2f, thr max: %6.2f thr cruis: %6.2f pt min: %6.2f, pt max: %6.2f\n",
	// 	et._DT, pitch, baro_altitude, hgt_dem, EAS_dem, indicated_airspeed, EAS2TAS, (climbOutDem) ? "climb" : "level", ptchMinCO, throttle_min, throttle_max, throttle_cruise, pitch_limit_min, pitch_limit_max);

	// Update the speed estimate using a 2nd order complementary filter
	ECL_tecs_update_speed(EAS_dem, indicated_airspeed, et._indicated_airspeed_min, et._indicated_airspeed_max, EAS2TAS);

	// Convert inputs
	et._THRmaxf  = throttle_max;
	et._THRminf  = throttle_min;
	et._PITCHmaxf = pitch_limit_max;
	et._PITCHminf = pitch_limit_min;
	et._climbOutDem = climbOutDem;

	// initialise selected states and variables if DT > 1 second or in climbout
	ECL_tecs_initialise_states(pitch, throttle_cruise, baro_altitude, ptchMinCO);

	// Calculate Specific Total Energy Rate Limits
	ECL_tecs_update_STE_rate_lim();

	// Calculate the speed demand
	ECL_tecs_update_speed_demand();

	// Calculate the height demand
	ECL_tecs_update_height_demand(hgt_dem);

	// Detect underspeed condition
	ECL_tecs_detect_underspeed();

	// Calculate specific energy quantitiues
	ECL_tecs_update_energies();

	// Calculate throttle demand
	ECL_tecs_update_throttle(throttle_cruise, rotMat);

	// Detect bad descent due to demanded airspeed being too high
	ECL_tecs_detect_bad_descent();

	// Calculate pitch demand
	ECL_tecs_update_pitch();

    // Write internal variables to the log_tuning structure. This
    // structure will be logged in dataflash at 10Hz
	// log_tuning.hgt_dem  = et._hgt_dem_adj;
	// log_tuning.hgt      = et._integ3_state;
	// log_tuning.dhgt_dem = et._hgt_rate_dem;
	// log_tuning.dhgt     = et._integ2_state;
	// log_tuning.spd_dem  = et._TAS_dem_adj;
	// log_tuning.spd      = et._integ5_state;
	// log_tuning.dspd     = et._vel_dot;
	// log_tuning.ithr     = et._integ6_state;
	// log_tuning.iptch    = et._integ7_state;
	// log_tuning.thr      = et._throttle_dem;
	// log_tuning.ptch     = et._pitch_dem;
	// log_tuning.dspd_dem = et._TAS_rate_dem;
}
