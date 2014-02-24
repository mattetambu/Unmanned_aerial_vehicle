/**
 * @file safety.h
 *
 * Safety topic to pass safety state from px4io driver to commander
 * This concerns only the safety button of the px4io but has nothing to do
 * with arming/disarming.
 */

#ifndef TOPIC_SAFETY_H
#define TOPIC_SAFETY_H

	#include <stdint.h>
	#include "../ORB.h"
	#include "../../uav_library/common.h"

	struct safety_s {
		bool_t safety_switch_available;		/**< Set to true if a safety switch is connected */
		bool_t safety_off;					/**< Set to true if safety is off */
	};

	ORB_DECLARE(safety);

#endif
