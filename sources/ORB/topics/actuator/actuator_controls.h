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

	/**
	 * @addtogroup topics
	 * @{
	 */

	struct actuator_controls_s {
		double aileron;
		double elevator;
		double rudder;
		double throttle;
		double flaps;
	};

	/**
	 * @}
	 */

	ORB_DECLARE(actuator_controls);

#endif
