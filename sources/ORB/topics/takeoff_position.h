/**
 * @file takeoff_position.h
 * Definition of the GPS takeoff position ORB topic.
 *
 */

#ifndef TOPIC_TAKEOFF_POSITION_H_
#define TOPIC_TAKEOFF_POSITION_H_

	#include <stdint.h>
	#include "../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * GPS takeoff position in WGS84 coordinates.
	 */
	struct takeoff_position_s
	{
		uint64_t time_gps_usec;		/**< Timestamp (microseconds in GPS format), this is the timestamp from the gps module */
		
		double latitude;			/**< Latitude */
		double longitude;			/**< Longitude */
		float altitude;				/**< Altitude above MSL */
		
		float eph_m;				/**< GPS HDOP horizontal dilution of position in m */
		float epv_m;				/**< GPS VDOP horizontal dilution of position in m */
		float s_variance_m_s;		/**< speed accuracy estimate m/s */
		float p_variance_m;			/**< position accuracy estimate m */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(takeoff_position);

#endif
