/**
 * @file navigation_capabilities.h
 *
 * Definition of navigation capabilities uORB topic.
 */

#ifndef TOPIC_NAVIGATION_CAPABILITIES_H_
#define TOPIC_NAVIGATION_CAPABILITIES_H_

	#include "../ORB.h"
	#include <stdint.h>

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * navigation_capabilities
	 */
	struct navigation_capabilities_s {
		float turn_distance;    /**< the optimal distance to a waypoint to switch to the next */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(navigation_capabilities);

#endif
