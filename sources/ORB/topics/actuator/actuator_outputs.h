/**
 * @file actuator_outputs.h
 *
 * Actuator output values.
 *
 * Values published to these topics are the outputs of the vehicle control
 * system and mixing process (control mixing system)as sent 
 * to the actuators (servos, motors, etc.) that operate the vehicle
 * 
 * Each topic can be published by a single output driver
 */
 
 
#ifndef TOPIC_ACTUATOR_OUTPUTS_H
#define TOPIC_ACTUATOR_OUTPUTS_H

	#include <stdint.h>
	#include "../../ORB.h"
	#include "../../../uav_library/common.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	struct actuator_outputs_s {
		double aileron;
		double elevator;
		double rudder;
		double throttle;
		
		bool_t valid_outputs;
	};

	/**
	 * @}
	 */
	
	ORB_DECLARE(actuator_outputs);

#endif
