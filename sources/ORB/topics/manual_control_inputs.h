/**
 * @file manual_control_inputs.h
 * Definition of the manual control inputs ORB topic.
 */

#ifndef TOPIC_MANUAL_CONTROL_INPUTS_H_
#define TOPIC_MANUAL_CONTROL_INPUTS_H_

	#include <stdint.h>
	#include "../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	struct manual_control_inputs_s {
		float roll;				 	/**< ailerons roll / roll rate input */
		float pitch;				/**< elevator / pitch / pitch rate */
		float yaw;					/**< rudder / yaw rate / yaw */
		float throttle;				/**< throttle / collective thrust / altitude */

		float mode_switch;			/**< 2/3 position switch (mandatory): manual, [support,] auto */
		float second_switch;		/**< 2/3 position switch (optional) */
		float third_switch;			/**< 2/3 position switch (optional) */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(manual_control_inputs);

#endif
