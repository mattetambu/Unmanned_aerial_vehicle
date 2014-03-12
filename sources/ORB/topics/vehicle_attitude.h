/**
 * @file vehicle_attitude.h
 * Definition of the attitude ORB topic.
 */

#ifndef TOPIC_VEHICLE_ATTITUDE_H_
#define TOPIC_VEHICLE_ATTITUDE_H_

	#include <stdint.h>
	#include "../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Attitude in NED body frame in SI units.
	 */
	struct vehicle_attitude_s {
		float roll;			/**< Roll angle (rad, Tait-Bryan, NED) */
		float pitch;		/**< Pitch angle (rad, Tait-Bryan, NED) */
		float yaw;			/**< Yaw angle (rad, Tait-Bryan, NED) */
		
		float roll_rate;	/**< Roll angular speed (rad/s, Tait-Bryan, NED) */
		float pitch_rate;	/**< Pitch angular speed (rad/s, Tait-Bryan, NED) */
		float yaw_rate;		/**< Yaw angular speed (rad/s, Tait-Bryan, NED) */
		
		float roll_acc;		/**< Roll angular accelration (rad/s2, Tait-Bryan, NED) */
		float pitch_acc;	/**< Pitch angular acceleration (rad/s2, Tait-Bryan, NED) */
		float yaw_acc;		/**< Yaw angular acceleration (rad/s2, Tait-Bryan, NED) */
		
		float rate_offsets[3];	/**< Offsets of the body angular rates from zero */
		float R[3][3];			/**< Rotation matrix body to world, (Tait-Bryan, NED) */
		float q[4];				/**< Quaternion (NED) */
		int R_valid;			/**< Rotation matrix valid */
		int q_valid;			/**< Quaternion valid */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_attitude);

#endif
