/**
 * @file actuator_armed.h
 *
 * Actuator armed topic
 *
 */

#ifndef TOPIC_ACTUATOR_ARMED_H
#define TOPIC_ACTUATOR_ARMED_H

	#include <stdint.h>
	#include "../../ORB.h"
	#include "../../../uav_library/common.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * global 'actuator output is live' control
	 */
	struct actuator_armed_s {
		bool_t armed;				/**< Set to true if system is armed */
		bool_t ready_to_arm;		/**< Set to true if system is ready to be armed */
		bool_t lockdown;			/**< Set to true if actuators are forced to being disabled (due to emergency or HIL) */
	};

	/**
	 * @}
	 */
	
	ORB_DECLARE(actuator_armed);

#endif
