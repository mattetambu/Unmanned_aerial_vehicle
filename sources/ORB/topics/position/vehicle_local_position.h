/**
 * @file vehicle_local_position.h
 * Definition of the local fused NED position uORB topic.
 */

#ifndef TOPIC_VEHICLE_LOCAL_POSITION_H_
#define TOPIC_VEHICLE_LOCAL_POSITION_H_

	#include <stdint.h>
	#include "../../ORB.h"
	#include "../../../uav_library/common.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Fused local position in NED.
	 */
	struct vehicle_local_position_s
	{
		uint64_t timestamp;		/**< Time of this estimate, in microseconds since system start */
		bool_t xy_valid;			/**< true if x and y are valid */
		bool_t z_valid;			/**< true if z is valid */
		bool_t v_xy_valid;		/**< true if vy and vy are valid */
		bool_t v_z_valid;			/**< true if vz is valid */
		/* Position in local NED frame */
		float x;				/**< X position in meters in NED earth-fixed frame */
		float y;				/**< X position in meters in NED earth-fixed frame */
		float z;				/**< Z position in meters in NED earth-fixed frame (negative altitude) */
		/* Velocity in NED frame */
		float vx; 				/**< Ground X Speed (Latitude), m/s in NED */
		float vy;				/**< Ground Y Speed (Longitude), m/s in NED */
		float vz;				/**< Ground Z Speed (Altitude), m/s	in NED */
		/* Heading */
		float yaw;
		/* Reference position in GPS / WGS84 frame */
		bool_t xy_global;			/**< true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon) */
		bool_t z_global;			/**< true if z is valid and has valid global reference (ref_alt) */
		uint64_t ref_timestamp;	/**< Time when reference position was set */
		int32_t ref_lat;		/**< Reference point latitude in 1E7 degrees */
		int32_t ref_lon;		/**< Reference point longitude in 1E7 degrees */
		float ref_alt;			/**< Reference altitude AMSL in meters, MUST be set to current (not at reference point!) ground level */
		bool_t landed;			/**< true if vehicle is landed */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_local_position);

#endif
