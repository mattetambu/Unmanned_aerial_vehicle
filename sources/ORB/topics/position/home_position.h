/**
 * @file home_position.h
 * Definition of the GPS home position ORB topic.
 *
 */

#ifndef TOPIC_HOME_POSITION_H_
#define TOPIC_HOME_POSITION_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * GPS home position in WGS84 coordinates.
	 */
	struct home_position_s
	{
		uint64_t time_gps_usec;		/**< Timestamp (microseconds in GPS format), this is the timestamp from the gps module */
		
		double latitude;			/**< Latitude */
		double longitude;			/**< Longitude */
		float altitude;				/**< Altitude above MSL */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(home_position);

#endif
