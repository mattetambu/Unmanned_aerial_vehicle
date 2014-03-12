/**
 * @file vehicle_attitude_setpoint.h
 * Definition of the vehicle attitude setpoint ORB topic.
 */

#ifndef TOPIC_VEHICLE_ATTITUDE_SETPOINT_H_
#define TOPIC_VEHICLE_ATTITUDE_SETPOINT_H_

	#include <stdint.h>
	#include "../../ORB.h"
	#include "../../../uav_library/common.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * vehicle attitude setpoint
	 */
	struct vehicle_attitude_setpoint_s
	{
		float roll_body;			/**< body angle in NED frame */
		float pitch_body;			/**< body angle in NED frame */
		float yaw_body;				/**< body angle in NED frame */
		//float body_valid;			/**< Set to true if body angles are valid */
		float roll_rate;			/**< body angular rates in NED frame */
		float pitch_rate;			/**< body angular rates in NED frame */
		float yaw_rate;				/**< body angular rates in NED frame */

		float R_body[9];			/**< Rotation matrix describing the setpoint as rotation from the current body frame */
		bool_t R_valid;				/**< Set to true if rotation matrix is valid */

		//! For quaternion-based attitude control
		float q_d[4];				/** Desired quaternion for quaternion control */
		bool_t q_d_valid;			/**< Set to true if quaternion vector is valid */
		float q_e[4];				/** Attitude error in quaternion */
		bool_t q_e_valid;			/**< Set to true if quaternion error vector is valid */

		float thrust;				/**< Thrust in Newton the power system should generate */
		bool_t roll_reset_integral;	/**< Reset roll integral part (navigation logic change) */

	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_attitude_setpoint);

#endif
