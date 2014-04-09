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


	#define NUM_ACTUATOR_OUTPUTS		16
	#define NUM_ACTUATOR_OUTPUT_GROUPS	4	/**< for sanity checking */

	/* output sets with pre-defined applications */
	#define ORB_ID_VEHICLE_CONTROLS		ORB_ID(actuator_outputs_0)

	/**
	 * @addtogroup topics
	 * @{
	 */

	typedef struct actuator_outputs_s {
		float	output[NUM_ACTUATOR_OUTPUTS];		/**< output data, in natural output units */
		unsigned noutputs;					/**< valid outputs */
	} actuator_outputs_s;

	/**
	 * @}
	 */
	
	/* actuator output sets; this list can be expanded as more drivers emerge */
	ORB_DECLARE_MANY(actuator_outputs_0, struct actuator_outputs_s);
	ORB_DECLARE_MANY(actuator_outputs_1, struct actuator_outputs_s);
	ORB_DECLARE_MANY(actuator_outputs_2, struct actuator_outputs_s);
	ORB_DECLARE_MANY(actuator_outputs_3, struct actuator_outputs_s);

#endif
