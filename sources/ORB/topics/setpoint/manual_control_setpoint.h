/**
 * @file manual_control_setpoint.h
 * Definition of the manual control inputs ORB topic.
 */

#ifndef TOPIC_MANUAL_CONTROL_SETPOINT_H_
#define TOPIC_MANUAL_CONTROL_SETPOINT_H_

	#include <stdint.h>
	#include "../../ORB.h"

	typedef enum {
		mode_switch_manual = 0,
		mode_switch_support,
		mode_switch_auto
	} mode_switch_pos_t;

	// do not change the order (it matches the combobox entries order in GUI)
	typedef enum {
		secondary_switch_none = 0,
		secondary_switch_manual,
		secondary_switch_stabilize,
		secondary_switch_fly_by_wire,
		secondary_switch_easy,
		secondary_switch_loiter,
		secondary_switch_rtl,
		secondary_switch_takeoff,
		secondary_switch_land,
		secondary_switch_mission
	} secondary_switch_pos_t;


	/**
	 * @addtogroup topics
	 * @{
	 */

	struct manual_control_setpoint_s {
		float roll;				 	/**< ailerons roll / roll rate input */
		float pitch;				/**< elevator / pitch / pitch rate */
		float yaw;					/**< rudder / yaw rate / yaw */
		float throttle;				/**< throttle / collective thrust / altitude */
		float flaps;				/**< flaps */

		bool_t want_to_arm;

		mode_switch_pos_t mode_switch;				/**< 2/3 position switch (mandatory): manual, [support,] auto */
		secondary_switch_pos_t second_switch;		/**< 2/3 position switch (optional) */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(manual_control_setpoint);

#endif
