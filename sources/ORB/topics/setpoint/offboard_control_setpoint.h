/**
 * @file offboard_control_setpoint.h
 * Definition of the manual_control_setpoint uORB topic.
 */

#ifndef TOPIC_OFFBOARD_CONTROL_SETPOINT_H_
#define TOPIC_OFFBOARD_CONTROL_SETPOINT_H_

	#include <stdint.h>
	#include "../../ORB.h"
	#include "../../../uav_library/common.h"

	/**
	 * Off-board control inputs.
	 * 
	 * Typically sent by a ground control station / joystick or by
	 * some off-board controller via C or SIMULINK.
	 */
	enum OFFBOARD_CONTROL_MODE
	{
		OFFBOARD_CONTROL_MODE_DIRECT = 0,
		OFFBOARD_CONTROL_MODE_DIRECT_RATES = 1,
		OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE = 2,
		OFFBOARD_CONTROL_MODE_DIRECT_VELOCITY = 3,
		OFFBOARD_CONTROL_MODE_DIRECT_POSITION = 4,
		OFFBOARD_CONTROL_MODE_ATT_YAW_RATE = 5,
		OFFBOARD_CONTROL_MODE_ATT_YAW_POS = 6,
		OFFBOARD_CONTROL_MODE_MULTIROTOR_SIMPLE = 7, /**< roll / pitch rotated aligned to the takeoff orientation, throttle stabilized, yaw pos */
	};

	/**
	 * @addtogroup topics
	 * @{
	 */
	
	/**< offboard control inputs */
	struct offboard_control_setpoint_s {
		enum OFFBOARD_CONTROL_MODE mode;		 /**< The current control inputs mode */
		bool_t armed;		/**< Armed flag set, yes / no */

		float p1;		/**< ailerons roll / roll rate input */
		float p2;		/**< elevator / pitch / pitch rate */
		float p3;		/**< rudder / yaw rate / yaw */
		float p4;		/**< throttle / collective thrust / altitude */

		float override_mode_switch;

		float aux1_cam_pan_flaps;
		float aux2_cam_tilt;
		float aux3_cam_zoom;
		float aux4_cam_roll;
	};
	
	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(offboard_control_setpoint);

#endif
