/**
 * @file vehicle_global_velocity_setpoint.h
 * Definition of the global velocity setpoint uORB topic.
 */

#ifndef TOPIC_VEHICLE_GLOBAL_VELOCITY_SETPOINT_H_
#define TOPIC_VEHICLE_GLOBAL_VELOCITY_SETPOINT_H_

	#include <stdint.h>
	#include "../../ORB.h"


	/**
	 * @addtogroup topics
	 * @{
	 */

	/**< Velocity setpoint in NED frame */
	struct vehicle_global_velocity_setpoint_s
	{
		float vx;		/**< in m/s NED			  		*/
		float vy;		/**< in m/s NED			  		*/
		float vz;		/**< in m/s NED			  		*/
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_global_velocity_setpoint);

#endif
