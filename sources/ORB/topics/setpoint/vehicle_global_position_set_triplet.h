/**
 * @file vehicle_global_position_setpoint.h
 * Definition of the global WGS84 position setpoint uORB topic.
 */

#ifndef TOPIC_VEHICLE_GLOBAL_POSITION_SET_TRIPLET_H_
#define TOPIC_VEHICLE_GLOBAL_POSITION_SET_TRIPLET_H_

	#include <stdint.h>
	#include "../../ORB.h"
	#include "../../../uav_library/common.h"

	#include "../setpoint/vehicle_global_position_setpoint.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Global position setpoint triplet in WGS84 coordinates.
	 *
	 * This are the three next waypoints (or just the next two or one).
	 */
	struct vehicle_global_position_set_triplet_s
	{
		bool_t previous_valid;					/**< flag indicating previous position is valid */
		bool_t next_valid;					/**< flag indicating next position is valid */

		struct vehicle_global_position_setpoint_s previous;
		struct vehicle_global_position_setpoint_s current;
		struct vehicle_global_position_setpoint_s next;
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_global_position_set_triplet);

#endif
