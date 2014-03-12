/**
 * @file vehicle_rates_setpoint.h
 * Definition of the vehicle rates setpoint topic
 */

#ifndef TOPIC_VEHICLE_RATES_SETPOINT_H_
#define TOPIC_VEHICLE_RATES_SETPOINT_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */
	struct vehicle_rates_setpoint_s
	{
		float roll;			/**< body angular rates in NED frame		*/
		float pitch;		/**< body angular rates in NED frame		*/
		float yaw;			/**< body angular rates in NED frame		*/
		float thrust;		/**< thrust normalized to 0..1			*/
	};

	 /**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_rates_setpoint);

#endif
