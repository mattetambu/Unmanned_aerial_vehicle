/**
 * @file airspeed.h
 *
 * Definition of airspeed topic
 */

#ifndef TOPIC_AIRSPEED_H_
#define TOPIC_AIRSPEED_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Airspeed
	 */
	struct airspeed_s {
		float		indicated_airspeed_m_s;		/**< indicated airspeed in meters per second, -1 if unknown	 */
		float		true_airspeed_m_s;			/**< true airspeed in meters per second, -1 if unknown */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(airspeed);

#endif
