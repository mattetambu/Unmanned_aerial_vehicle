/**
 * @file vehicle_speed_setpoint.h
 * Definition of the speed (bodyframe and global) setpoint uORB topic.
 */

#ifndef TOPIC_VEHICLE_SPEED_SETPOINT_H_
#define TOPIC_VEHICLE_SPEED_SETPOINT_H_

	#include "../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */
	 
	/**
	 * Speed in bodyframe and in NED frame to go to
	 */
	struct vehicle_speed_setpoint_s
	{
		float vx_body;				/**< in m/s */
		float vy_body;				/**< in m/s */
		// float vz_body;			/**< in m/s */
		
		float vx_global;			/**< in m/s NED */
		float vy_global;			/**< in m/s NED */
		float vz_global;			/**< in m/s NED */
		
		float thrust_sp;
		float yaw_sp;			/**< in radians -PI..+PI */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_speed_setpoint);

#endif
