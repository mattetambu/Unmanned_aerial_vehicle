/**
 * @file actuator_effective_controls.h
 *
 * Actuator control topics - mixer inputs.
 *
 * Values published to these topics are the outputs of the vehicle control
 * system and mixing process; they are the control-scale values that are
 * then fed to the actual actuator driver.
 *
 * Each topic can be published by a single controller
 */

#ifndef TOPIC_ACTUATOR_EFFECTIVE_CONTROLS_H
#define TOPIC_ACTUATOR_EFFECTIVE_CONTROLS_H

	#include <stdint.h>
	#include "../../ORB.h"
	#include "actuator_controls.h"

	#define NUM_ACTUATOR_CONTROLS_EFFECTIVE		NUM_ACTUATOR_CONTROLS
	#define NUM_ACTUATOR_CONTROL_GROUPS_EFFECTIVE	NUM_ACTUATOR_CONTROL_GROUPS	/**< for sanity checking */

	/* control sets with pre-defined applications */
	#define ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE	ORB_ID(actuator_effective_controls_0)

	/**
	 * @addtogroup topics
	 * @{
	 */

	struct actuator_effective_controls_s {
		float	control[NUM_ACTUATOR_CONTROLS_EFFECTIVE];
	};

	/**
	 * @}
	 */

	/* actuator control sets; this list can be expanded as more controllers emerge */
	ORB_DECLARE_MANY(actuator_effective_controls_0, struct actuator_effective_controls_s);
	ORB_DECLARE_MANY(actuator_effective_controls_1, struct actuator_effective_controls_s);
	ORB_DECLARE_MANY(actuator_effective_controls_2, struct actuator_effective_controls_s);
	ORB_DECLARE_MANY(actuator_effective_controls_3, struct actuator_effective_controls_s);

#endif 
