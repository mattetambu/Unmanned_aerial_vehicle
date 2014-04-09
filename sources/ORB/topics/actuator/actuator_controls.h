/**
 * @file actuator_controls.h
 *
 * Actuator control topics - mixer inputs.
 *
 * Values published to these topics are the outputs of the vehicle control
 * system, and are expected to be mixed and used to drive the actuators
 * (servos, speed controls, etc.) that operate the vehicle
 *
 * Each topic can be published by a single controller
 */

#ifndef TOPIC_ACTUATOR_CONTROLS_H
#define TOPIC_ACTUATOR_CONTROLS_H

	#include <stdint.h>
	#include "../../ORB.h"


	#define NUM_ACTUATOR_CONTROLS		8
	#define NUM_ACTUATOR_CONTROL_GROUPS	4	/**< for sanity checking */

	/* control sets with pre-defined applications */
	#define ORB_ID_VEHICLE_ATTITUDE_CONTROLS	ORB_ID(actuator_controls_0)

	/**
	 * @addtogroup topics
	 * @{
	 */

	typedef struct actuator_controls_s {
		float	control[NUM_ACTUATOR_CONTROLS];
	} actuator_controls_s;

	/**
	 * @}
	 */

	/* actuator control sets; this list can be expanded as more controllers emerge */
	ORB_DECLARE_MANY(actuator_controls_0, struct actuator_controls_s);
	ORB_DECLARE_MANY(actuator_controls_1, struct actuator_controls_s);
	ORB_DECLARE_MANY(actuator_controls_2, struct actuator_controls_s);
	ORB_DECLARE_MANY(actuator_controls_3, struct actuator_controls_s);

#endif
