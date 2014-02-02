/**
 * @file vehicle_global_position.h
 * Definition of the global fused WGS84 position ORB topic.
 */

#ifndef TOPIC_VEHICLE_GLOBAL_POSITION_H_
#define TOPIC_VEHICLE_GLOBAL_POSITION_H_

	#include <stdint.h>
	#include "../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	 /**
	 * Fused global position in WGS84.
	 *
	 * This struct contains the system's believ about its position. It is not the raw GPS
	 * measurement (@see vehicle_gps_position). This topic is usually published by the position
	 * estimator, which will take more sources of information into account than just GPS,
	 * e.g. control inputs of the vehicle in a Kalman-filter implementation.
	 */
	struct vehicle_global_position_s
	{
		uint64_t time_gps_usec;		/**< Timestamp (microseconds in GPS format), this is the timestamp from the gps module */
		int valid;					/**< true if position satisfies validity criteria of estimator */
		float yaw;					/**< Compass heading in radians -PI..+PI. */
		
		double latitude;			/**< Latitude */
		double longitude;			/**< Longitude */
		float altitude;				/**< Altitude above MSL */
		//float relative_altitude;	/**< Altitude above home position in meters */
		float ground_level;			/**< Must be set to current ground level */

		float vx;					/**< Ground X velocity, m/s in NED */
		float vy;					/**< Ground Y velocity, m/s in NED */
		float vz;					/**< Ground Z velocity, m/s in NED */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_global_position);

#endif
