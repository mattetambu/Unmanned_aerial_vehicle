/**
 * @file ECL_l1_position_controller.h
 * Implementation of L1 position control.
 *
 *
 * Acknowledgements and References:
 *
 *    This implementation has been built for PX4 based on the original
 *    publication from [1] and does include a lot of the ideas (not code)
 *    from [2].
 *
 *
 *    [1] S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
 *    Proceedings of the AIAA Guidance, Navigation and Control
 *    Conference, Aug 2004. AIAA-2004-4900.
 *
 *    [2] Paul Riseborough, Brandon Jones and Andrew Tridgell, L1 control for APM. Aug 2013.
 *     - Explicit control over frequency and damping
 *     - Explicit control over track capture angle
 *     - Ability to use loiter radius smaller than L1 length
 *     - Modified to use PD control for circle tracking to enable loiter radius less than L1 length
 *     - Modified to enable period and damping of guidance loop to be set explicitly
 *     - Modified to provide explicit control over capture angle
 *
 */


/**
 * L1 Nonlinear Guidance Logic
 */
#ifndef ECL_L1_POSITION_CONTROLLER_H
#define ECL_L1_POSITION_CONTROLLER_H

	#include "../../uav_library/common.h"
	#include "../../uav_library/math/Dcm.h"
	#include "../../uav_library/math/Vector3f.h"


	void ECL_l1_position_controller_init ();
	float ECL_l1_position_controller_nav_bearing();
	float ECL_l1_position_controller_nav_lateral_acceleration_demand();
	float ECL_l1_position_controller_bearing_error();
	float ECL_l1_position_controller_target_bearing();
	float ECL_l1_position_controller_nav_roll();

	float ECL_l1_position_controller_crosstrack_error();
	bool_t ECL_l1_position_controller_reached_loiter_target();
	bool_t ECL_l1_position_controller_circle_mode();

	float ECL_l1_position_controller_switch_distance(float waypoint_switch_radius);
	int ECL_l1_position_controller_navigate_waypoints(Vector2f *vector_A, Vector2f *vector_B, Vector2f *vector_curr_position, Vector2f *ground_speed);
	int ECL_l1_position_controller_navigate_loiter(Vector2f *vector_A, Vector2f *vector_curr_position, float radius, int8_t loiter_direction, Vector2f *ground_speed_vector);
	int ECL_l1_position_controller_navigate_heading(float navigation_heading, float current_heading, Vector2f *ground_speed);
	void ECL_l1_position_controller_navigate_level_flight(float current_heading);

	void ECL_l1_position_controller_set_l1_period(float period);
	void ECL_l1_position_controller_set_l1_damping(float damping);
	void ECL_l1_position_controller_set_l1_roll_limit(float roll_lim_rad);

#endif /* ECL_L1_POS_CONTROLLER_H */
