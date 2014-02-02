/**
 * @file vehicle_global_position_setpoint.h
 * Definition of the global WGS84 position setpoint uORB topic.
 */

#ifndef TOPIC_VEHICLE_GLOBAL_POSITION_SETPOINT_H_
#define TOPIC_VEHICLE_GLOBAL_POSITION_SETPOINT_H_

	#include <stdint.h>
	#include "../ORB.h"
	//#include "mission.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Global position setpoint in WGS84 coordinates.
	 *
	 * This is the position the MAV is heading towards. If it of type loiter,
	 * the MAV is circling around it with the given loiter radius in meters.
	 */
	struct vehicle_global_position_setpoint_s
	{
		double latitude;			/**< Latitude */
		double longitude;			/**< Longitude */
		float altitude;				/**< Altitude above MSL */
		bool altitude_is_relative;	/**< true if altitude is relative from start point */
		float yaw;					/**< in radians NED -PI..+PI */
		
		bool loitering_enabled;		/**< true if loitering is enabled */
		float loiter_radius;		/**< loiter radius in meters, 0 for a VTOL to hover */
		int8_t loiter_direction;	/**< 1: positive / clockwise, -1, negative */
		
		float turn_distance_xy;		/**< The distance on the plane which will mark this as reached */
		float turn_distance_z;		/**< The distance in Z direction which will mark this as reached */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_global_position_setpoint);

#endif
