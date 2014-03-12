/**
 * @file ECL_l1_position_controller.h
 * Implementation of L1 position control.
 * Authors and acknowledgements in header.
 *
 */

#include "ECL_l1_position_controller.h"
#include "../../uav_library/common.h"
#include "../../uav_library/geo/geo.h"
#include "../../uav_library/time/drv_time.h"
#include "../../uav_library/math/limits.h"
#include "../../uav_library/math/Dcm.h"
#include "../../uav_library/math/Vector2f.h"


struct ECL_l1_position_controller {
	float _lateral_accel;		///< Lateral acceleration setpoint in m/s^2
	float _L1_distance;		///< L1 lead distance, defined by period and damping
	bool_t _circle_mode;		///< flag for loiter mode
	float _nav_bearing;		///< bearing to L1 reference point
	float _bearing_error;		///< bearing error
	float _crosstrack_error;	///< crosstrack error in meters
	float _target_bearing;		///< the heading setpoint

	float _L1_period;		///< L1 tracking period in seconds
	float _L1_damping;		///< L1 damping ratio
	float _L1_ratio;		///< L1 ratio for navigation
	float _K_L1;			///< L1 control gain for _L1_damping
	float _heading_omega;		///< Normalized frequency

	float _roll_lim_rad;  ///<maximum roll angle
} elpc;


// function prototypes
int ECL_l1_position_controller_get_local_planar_vector(Vector2f *origin, Vector2f *result, Vector2f *target);


void ECL_l1_position_controller_init ()
{
	elpc._L1_period = 25;
	elpc._L1_damping = 0.75f;
}

/**
 * Returns true if following a circle (loiter)
 */
bool_t ECL_l1_position_controller_circle_mode() {
	return elpc._circle_mode;
}


/**
 * Set the L1 period.
 */
void ECL_l1_position_controller_set_l1_period(float period) {
	elpc._L1_period = period;
	/* calculate the ratio introduced in [2] */
	elpc._L1_ratio = 1.0f / M_PI * elpc._L1_damping * elpc._L1_period;
	/* calculate normalized frequency for heading tracking */
	elpc._heading_omega = sqrtf(2.0f) * M_PI / elpc._L1_period;
}


/**
 * Set the L1 damping factor.
 *
 * The original publication recommends a default of sqrt(2) / 2 = 0.707
 */
void ECL_l1_position_controller_set_l1_damping(float damping) {
	elpc._L1_damping = damping;
	/* calculate the ratio introduced in [2] */
	elpc._L1_ratio = 1.0f / M_PI * elpc._L1_damping * elpc._L1_period;
	/* calculate the L1 gain (following [2]) */
	elpc._K_L1 = 4.0f * elpc._L1_damping * elpc._L1_damping;
}


/**
 * Set the maximum roll angle output in radians
 *
 */
void ECL_l1_position_controller_set_l1_roll_limit(float roll_lim_rad) {
	elpc._roll_lim_rad = roll_lim_rad;
}


/**
 * Get roll angle setpoint for fixed wing.
 *
 * @return Roll angle (in NED frame)
 */
float ECL_l1_position_controller_nav_roll()
{
	float ret = atanf(elpc._lateral_accel * 1.0f / CONSTANTS_ONE_G);
	ret = constrain(ret, -elpc._roll_lim_rad, elpc._roll_lim_rad);
	return ret;
}

/**
 * Get lateral acceleration demand.
 *
 * @return Lateral acceleration in m/s^2
 */
float ECL_l1_position_controller_nav_lateral_acceleration_demand()
{
	return elpc._lateral_accel;
}

/**
 * The current target bearing
 *
 * @return bearing angle (-pi..pi, in NED frame)
 */
float ECL_l1_position_controller_nav_bearing()
{
	return _wrap_pi(elpc._nav_bearing);
}

/**
 * Heading error.
 *
 * The heading error is either compared to the current track
 * or to the tangent of the current loiter radius.
 */
float ECL_l1_position_controller_bearing_error()
{
	return elpc._bearing_error;
}

/**
 * Bearing from aircraft to current target.
 *
 * @return bearing angle (-pi..pi, in NED frame)
 */
float ECL_l1_position_controller_target_bearing()
{
	return elpc._target_bearing;
}

/**
 * Get the switch distance
 *
 * This is the distance at which the system will
 * switch to the next waypoint. This depends on the
 * period and damping
 *
 * @param waypoint_switch_radius The switching radius the waypoint has set.
 */
float ECL_l1_position_controller_switch_distance(float wp_radius)
{
	/* following [2], switching on L1 distance */
	return max(wp_radius, elpc._L1_distance);
}

/**
 * Returns true if the loiter waypoint has been reached
 */
bool_t ECL_l1_position_controller_reached_loiter_target(void)
{
	return elpc._circle_mode;
}

/**
 * Get the current crosstrack error.
 *
 * @return Crosstrack error in meters.
 */
float ECL_l1_position_controller_crosstrack_error(void)
{
	return elpc._crosstrack_error;
}


/**
 * Navigate between two waypoints
 *
 * Calling this function with two waypoints results in the
 * control outputs to fly to the line segment defined by
 * the points and once captured following the line segment.
 * This follows the logic in [1].
 *
 * @return sets elpc._lateral_accel setpoint
 */
int ECL_l1_position_controller_navigate_waypoints(Vector2f *vector_A, Vector2f *vector_B, Vector2f *vector_curr_position, Vector2f *ground_speed_vector)
{
	/* this follows the logic presented in [1] */
	float eta, xtrack_vel, ltrack_vel;
	float eta2, xtrackErr, sine_eta1, eta1;
	float ground_speed, distance_A_to_airplane, distance_A_to_B, alongTrackDist, AB_to_BP_bearing;

	float vector_curr_positionX, vector_curr_positionY, vector_BX, vector_BY;
	float vector_A_to_airplane_unitX, vector_A_to_airplane_unitY, vector_B_to_P_unitX, vector_B_to_P_unitY;
	float vector_ABX, vector_ABY;
	Vector2f vector_AB, vector_B_to_P_unit, vector_A_to_airplane, vector_A_to_airplane_unit, temp_vector;


	/* get the direction between the last (visited) and next waypoint */
	MATHLIB_ASSERT (Vector2f_getX (vector_curr_position, &vector_curr_positionX));
	MATHLIB_ASSERT (Vector2f_getY (vector_curr_position, &vector_curr_positionY));
	MATHLIB_ASSERT (Vector2f_getX (vector_B, &vector_BX));
	MATHLIB_ASSERT (Vector2f_getY (vector_B, &vector_BY));
	elpc._target_bearing = get_bearing_to_next_waypoint(vector_curr_positionX, vector_curr_positionY, vector_BX, vector_BY);

	/* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
	MATHLIB_ASSERT (Vector2f_length (ground_speed_vector, &ground_speed));
	ground_speed = max(ground_speed, 0.1f);

	/* calculate the L1 length required for the desired period */
	elpc._L1_distance = elpc._L1_ratio * ground_speed;

	/* calculate vector from A to B */
	MATHLIB_ASSERT (ECL_l1_position_controller_get_local_planar_vector(vector_A, &vector_AB, vector_B));

	/*
	 * check if waypoints are on top of each other. If yes,
	 * skip A and directly continue to B
	 */
	MATHLIB_ASSERT (Vector2f_length (&vector_AB, &distance_A_to_B));
	if (distance_A_to_B < 1.0e-6f) {
		MATHLIB_ASSERT (ECL_l1_position_controller_get_local_planar_vector(vector_curr_position, &vector_AB, vector_B));
	}

	Vector2f_normalize (&vector_AB);

	/* calculate the vector from waypoint A to the aircraft */
	MATHLIB_ASSERT (ECL_l1_position_controller_get_local_planar_vector(vector_A, &vector_A_to_airplane, vector_curr_position));

	/* calculate crosstrack error (output only) */
	MATHLIB_ASSERT (Vector2f_cross_Vector2f (&vector_AB, &elpc._crosstrack_error, &vector_A_to_airplane));

	/*
	 * If the current position is in a +-135 degree angle behind waypoint A
	 * and further away from A than the L1 distance, then A becomes the L1 point.
	 * If the aircraft is already between A and B normal L1 logic is applied.
	 */
	MATHLIB_ASSERT (Vector2f_length (&vector_A_to_airplane, &distance_A_to_airplane));
	MATHLIB_ASSERT (Vector2f_mul_Vector2f (&vector_A_to_airplane, &alongTrackDist, &vector_AB));

	/* estimate airplane position WRT to B */
	MATHLIB_ASSERT (ECL_l1_position_controller_get_local_planar_vector(vector_B, &vector_B_to_P_unit, vector_curr_position));
	MATHLIB_ASSERT (Vector2f_normalize (&vector_B_to_P_unit));
	
	/* calculate angle of airplane position vector relative to line) */

	// XXX this could probably also be based solely on the dot product
	float B2P_cross_AB, B2P_mul_AB;
	MATHLIB_ASSERT (Vector2f_cross_Vector2f (&vector_B_to_P_unit, &B2P_cross_AB, &vector_AB));
	MATHLIB_ASSERT (Vector2f_mul_Vector2f (&vector_B_to_P_unit, &B2P_mul_AB, &vector_AB));
	AB_to_BP_bearing = atan2f (B2P_cross_AB, B2P_mul_AB);

	/* extension from [2], fly directly to A */
	if (distance_A_to_airplane > elpc._L1_distance && alongTrackDist / max(distance_A_to_airplane , 1.0f) < -0.7071f) {

		/* calculate eta to fly to waypoint A */

		/* unit vector from waypoint A to current position */
		MATHLIB_ASSERT (Vector2f_init_Vector2f (&vector_A_to_airplane_unit, &vector_A_to_airplane));
		MATHLIB_ASSERT (Vector2f_normalize (&vector_A_to_airplane_unit));

		/* velocity across / orthogonal to line */
		MATHLIB_ASSERT (Vector2f_init_Vector2f (&temp_vector, &vector_A_to_airplane_unit));
		MATHLIB_ASSERT (Vector2f_change_sign (&temp_vector));
		MATHLIB_ASSERT (Vector2f_cross_Vector2f (ground_speed_vector, &xtrack_vel, &temp_vector));

		/* velocity along line */
		MATHLIB_ASSERT (Vector2f_mul_Vector2f (ground_speed_vector, &ltrack_vel, &temp_vector));
		eta = atan2f(xtrack_vel, ltrack_vel);

		/* bearing from current position to L1 point */
		MATHLIB_ASSERT (Vector2f_getX (&vector_A_to_airplane_unit, &vector_A_to_airplane_unitX));
		MATHLIB_ASSERT (Vector2f_getY (&vector_A_to_airplane_unit, &vector_A_to_airplane_unitY));
		elpc._nav_bearing = atan2f(-vector_A_to_airplane_unitY , -vector_A_to_airplane_unitX);

	/*
	 * If the AB vector and the vector from B to airplane point in the same
	 * direction, we have missed the waypoint. At +- 90 degrees we are just passing it.
	 */
	} else if (fabsf(AB_to_BP_bearing) < radians(100.0f)) {
		/*
		 * Extension, fly back to waypoint.
		 * 
		 * This corner case is possible if the system was following
		 * the AB line from waypoint A to waypoint B, then is
		 * switched to manual mode (or otherwise misses the waypoint)
		 * and behind the waypoint continues to follow the AB line.
		 */

		/* calculate eta to fly to waypoint B */
		
		/* velocity across / orthogonal to line */
		MATHLIB_ASSERT (Vector2f_init_Vector2f (&temp_vector, &vector_B_to_P_unit));
		MATHLIB_ASSERT (Vector2f_change_sign (&temp_vector));
		MATHLIB_ASSERT (Vector2f_cross_Vector2f (ground_speed_vector, &xtrack_vel, &temp_vector));

		/* velocity along line */
		MATHLIB_ASSERT (Vector2f_mul_Vector2f (ground_speed_vector, &ltrack_vel, &temp_vector));
		eta = atan2f(xtrack_vel, ltrack_vel);

		/* bearing from current position to L1 point */
		MATHLIB_ASSERT (Vector2f_getX (&vector_B_to_P_unit, &vector_B_to_P_unitX));
		MATHLIB_ASSERT (Vector2f_getY (&vector_B_to_P_unit, &vector_B_to_P_unitY));
		elpc._nav_bearing = atan2f(-vector_B_to_P_unitY , -vector_B_to_P_unitX);

	} else {

		/* calculate eta to fly along the line between A and B */

		/* velocity across / orthogonal to line */
		MATHLIB_ASSERT (Vector2f_cross_Vector2f (ground_speed_vector, &xtrack_vel, &vector_AB));

		/* velocity along line */
		MATHLIB_ASSERT (Vector2f_mul_Vector2f (ground_speed_vector, &ltrack_vel, &vector_AB));

		/* calculate eta2 (angle of velocity vector relative to line) */
		eta2 = atan2f(xtrack_vel, ltrack_vel);

		/* calculate eta1 (angle to L1 point) */
		MATHLIB_ASSERT (Vector2f_cross_Vector2f (&vector_A_to_airplane, &xtrackErr, &vector_AB));
		sine_eta1 = xtrackErr / max(elpc._L1_distance , 0.1f);

		/* limit output to 45 degrees */
		sine_eta1 = constrain(sine_eta1, -0.7071f, 0.7071f); //sin(pi/4) = 0.7071
		eta1 = asinf(sine_eta1);
		eta = eta1 + eta2;

		/* bearing from current position to L1 point */
		MATHLIB_ASSERT (Vector2f_getX (&vector_AB, &vector_ABX));
		MATHLIB_ASSERT (Vector2f_getY (&vector_AB, &vector_ABY));
		elpc._nav_bearing = atan2f(vector_ABY, vector_ABX) + eta1;

	}

	/* limit angle to +-90 degrees */
	eta = constrain(eta, (-M_PI) / 2.0f, +M_PI / 2.0f);
	elpc._lateral_accel = elpc._K_L1 * ground_speed * ground_speed / elpc._L1_distance * sinf(eta);

	/* flying to waypoints, not circling them */
	elpc._circle_mode = 0 /* false */;

	/* the bearing angle, in NED frame */
	elpc._bearing_error = eta;

	return 0;
}


/**
 * Navigate on an orbit around a loiter waypoint.
 *
 * This allow orbits smaller than the L1 length,
 * this modification was introduced in [2].
 *
 * @return sets elpc._lateral_accel setpoint
 */
int ECL_l1_position_controller_navigate_loiter(Vector2f *vector_A, Vector2f *vector_curr_position, float radius, int8_t loiter_direction, Vector2f *ground_speed_vector)
{
	/* the complete guidance logic in this section was proposed by [2] */

	/* calculate the gains for the PD loop (circle tracking) */
	float ground_speed, distance_A_to_airplane;
	float omega = (2.0f * M_PI / elpc._L1_period);
	float K_crosstrack = omega * omega;
	float K_velocity = 2.0f * elpc._L1_damping * omega;
	float vector_curr_positionX, vector_curr_positionY, vector_AX, vector_AY, vector_A_to_airplane_unitX, vector_A_to_airplane_unitY;
	Vector2f vector_A_to_airplane, vector_A_to_airplane_unit;

	/* update bearing to next waypoint */
	MATHLIB_ASSERT (Vector2f_getX (vector_curr_position, &vector_curr_positionX));
	MATHLIB_ASSERT (Vector2f_getY (vector_curr_position, &vector_curr_positionY));
	MATHLIB_ASSERT (Vector2f_getX (vector_A, &vector_AX));
	MATHLIB_ASSERT (Vector2f_getY (vector_A, &vector_AY));
	elpc._target_bearing = get_bearing_to_next_waypoint(vector_curr_positionX, vector_curr_positionY, vector_AX, vector_AY);

	/* ground speed, enforce minimum of 0.1 m/s to avoid singularities */
	MATHLIB_ASSERT (Vector2f_length (ground_speed_vector, &ground_speed));
	ground_speed = max(ground_speed, 0.1f);

	/* calculate the L1 length required for the desired period */
	elpc._L1_distance = elpc._L1_ratio * ground_speed;

	/* calculate the vector from waypoint A to current position */
	MATHLIB_ASSERT (ECL_l1_position_controller_get_local_planar_vector(vector_A, &vector_A_to_airplane, vector_curr_position));

	/* store the normalized vector from waypoint A to current position */
	MATHLIB_ASSERT (Vector2f_init_Vector2f (&vector_A_to_airplane_unit, &vector_A_to_airplane));
	MATHLIB_ASSERT (Vector2f_normalize (&vector_A_to_airplane_unit));

	/* calculate eta angle towards the loiter center */

	/* velocity across / orthogonal to line from waypoint to current position */
	float xtrack_vel_center;
	MATHLIB_ASSERT (Vector2f_cross_Vector2f (&vector_A_to_airplane_unit, &xtrack_vel_center, ground_speed_vector));
	/* velocity along line from waypoint to current position */

	float ltrack_vel_center;
	MATHLIB_ASSERT (Vector2f_mul_Vector2f (ground_speed_vector, &ltrack_vel_center, &vector_A_to_airplane_unit));
	ltrack_vel_center = -ltrack_vel_center;

	float eta = atan2f(xtrack_vel_center, ltrack_vel_center);
	/* limit eta to 90 degrees */
	eta = constrain(eta, -M_PI / 2.0f, +M_PI / 2.0f);

	/* calculate the lateral acceleration to capture the center point */
	float lateral_accel_sp_center = elpc._K_L1 * ground_speed * ground_speed / elpc._L1_distance * sinf(eta);

	/* for PD control: Calculate radial position and velocity errors */

	/* radial velocity error */
	float xtrack_vel_circle = -ltrack_vel_center;
	/* radial distance from the loiter circle (not center) */
	MATHLIB_ASSERT (Vector2f_length (&vector_A_to_airplane, &distance_A_to_airplane));
	float xtrack_err_circle = distance_A_to_airplane - radius;

	/* cross track error for feedback */
	elpc._crosstrack_error = xtrack_err_circle;

	/* calculate PD update to circle waypoint */
	float lateral_accel_sp_circle_pd = (xtrack_err_circle * K_crosstrack + xtrack_vel_circle * K_velocity);

	/* calculate velocity on circle / along tangent */
	float tangent_vel = xtrack_vel_center * loiter_direction;

	/* prevent PD output from turning the wrong way */
	if (tangent_vel < 0.0f) {
		lateral_accel_sp_circle_pd = max(lateral_accel_sp_circle_pd , 0.0f);
	}

	/* calculate centripetal acceleration setpoint */
	float lateral_accel_sp_circle_centripetal = tangent_vel * tangent_vel / max((0.5f * radius) , (radius + xtrack_err_circle));

	/* add PD control on circle and centripetal acceleration for total circle command */
	float lateral_accel_sp_circle = loiter_direction * (lateral_accel_sp_circle_pd + lateral_accel_sp_circle_centripetal);

	/*
	 * Switch between circle (loiter) and capture (towards waypoint center) mode when
	 * the commands switch over. Only fly towards waypoint if outside the circle.
	 */

	MATHLIB_ASSERT (Vector2f_getX (&vector_A_to_airplane_unit, &vector_A_to_airplane_unitX));
	MATHLIB_ASSERT (Vector2f_getY (&vector_A_to_airplane_unit, &vector_A_to_airplane_unitY));


	// XXX check switch over
	if ((lateral_accel_sp_center < lateral_accel_sp_circle && loiter_direction > 0 && xtrack_err_circle > 0.0f) |
		(lateral_accel_sp_center > lateral_accel_sp_circle && loiter_direction < 0 && xtrack_err_circle > 0.0f)) {
		elpc._lateral_accel = lateral_accel_sp_center;
		elpc._circle_mode = 0 /* false */;
		/* angle between requested and current velocity vector */
		elpc._bearing_error = eta;
		/* bearing from current position to L1 point */
		elpc._nav_bearing = atan2f(-vector_A_to_airplane_unitY , -vector_A_to_airplane_unitX);

	} else {
		elpc._lateral_accel = lateral_accel_sp_circle;
		elpc._circle_mode = 1 /* true */;
		elpc._bearing_error = 0.0f;
		/* bearing from current position to L1 point */
		elpc._nav_bearing = atan2f(-vector_A_to_airplane_unitY , -vector_A_to_airplane_unitX);
	}

	return 0;
}


/**
 * Navigate on a fixed bearing.
 *
 * This only holds a certain direction and does not perform cross
 * track correction. Helpful for semi-autonomous modes. Introduced
 * by [2].
 *
 * @return sets elpc._lateral_accel setpoint
 */
int ECL_l1_position_controller_navigate_heading(float navigation_heading, float current_heading, Vector2f *ground_speed_vector)
{
	/* the complete guidance logic in this section was proposed by [2] */

	float eta, ground_speed;

	/* 
	 * As the commanded heading is the only reference
	 * (and no crosstrack correction occurs),
	 * target and navigation bearing become the same
	 */
	elpc._target_bearing = elpc._nav_bearing = _wrap_pi(navigation_heading);
	eta = elpc._target_bearing - _wrap_pi(current_heading);
	eta = _wrap_pi(eta);

	/* consequently the bearing error is exactly eta: */
	elpc._bearing_error = eta;

	/* ground speed is the length of the ground speed vector */
	MATHLIB_ASSERT (Vector2f_length (ground_speed_vector, &ground_speed));

	/* adjust L1 distance to keep constant frequency */
	elpc._L1_distance = ground_speed / elpc._heading_omega;
	float omega_vel = ground_speed * elpc._heading_omega;

	/* not circling a waypoint */
	elpc._circle_mode = 0 /* false */;

	/* navigating heading means by definition no crosstrack error */
	elpc._crosstrack_error = 0;

	/* limit eta to 90 degrees */
	eta = constrain(eta, (-M_PI) / 2.0f, +M_PI / 2.0f);
	elpc._lateral_accel = 2.0f * sinf(eta) * omega_vel;

	return 0;
}


/**
 * Keep the wings level.
 *
 * This is typically needed for maximum-lift-demand situations,
 * such as takeoff or near stall. Introduced in [2].
 */
void ECL_l1_position_controller_navigate_level_flight(float current_heading)
{
	/* the logic in this section is trivial, but originally proposed by [2] */

	/* reset all heading / error measures resulting in zero roll */
	elpc._target_bearing = current_heading;
	elpc._nav_bearing = current_heading;
	elpc._bearing_error = 0;
	elpc._crosstrack_error = 0;
	elpc._lateral_accel = 0;

	/* not circling a waypoint when flying level */
	elpc._circle_mode = 0 /* false */;

}

/**
 * Convert a 2D vector from WGS84 to planar coordinates.
 *
 * This converts from latitude and longitude to planar
 * coordinates with (0,0) being at the position of ref and
 * returns a vector in meters towards wp.
 *
 * @param ref The reference position in WGS84 coordinates
 * @param wp The point to convert to into the local coordinates, in WGS84 coordinates
 * @return The vector in meters pointing from the reference position to the coordinates
 */
int ECL_l1_position_controller_get_local_planar_vector(Vector2f *origin, Vector2f *result, Vector2f *target)
{
	/* this is an approximation for small angles, proposed by [2] */
	float targetX, targetY, originX, originY;

	MATHLIB_ASSERT (Vector2f_getX (target, &targetX));
	MATHLIB_ASSERT (Vector2f_getY (target, &targetY));
	MATHLIB_ASSERT (Vector2f_getX (origin, &originX));
	MATHLIB_ASSERT (Vector2f_getY (origin, &originY));

	MATHLIB_ASSERT (Vector2f_init_zero (result));
	MATHLIB_ASSERT (Vector2f_setX (result, radians((targetX - originX))));
	MATHLIB_ASSERT (Vector2f_setY (result, radians((targetY - originY)*cosf(radians(originX)))));
	MATHLIB_ASSERT (Vector2f_mul_float (result, (float) CONSTANTS_RADIUS_OF_EARTH));

	return 0;
}

