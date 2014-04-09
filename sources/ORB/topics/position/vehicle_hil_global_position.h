/**
 * @file vehicle_hil_global_position.h
 * Definition of the ORB topic of the global position obtained from the simulator
 */

#ifndef TOPIC_VEHICLE_HIL_GLOBAL_POSITION_H_
#define TOPIC_VEHICLE_HIL_GLOBAL_POSITION_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	struct vehicle_hil_global_position_s
	{
		float latitude;			/**< Latitude in 1E7 degrees */
		float longitude;			/**< Longitude in 1E7 degrees */
		float altitude;				/**< Altitude above MSL in meters */
		float ground_level;			/**< Must be set to current ground level */

		float vx;					/**< Ground X velocity, m/s in NED */
		float vy;					/**< Ground Y velocity, m/s in NED */
		float vz;					/**< Ground Z velocity, m/s in NED */

		/* Heading */
		float yaw;
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_hil_global_position);

#endif
