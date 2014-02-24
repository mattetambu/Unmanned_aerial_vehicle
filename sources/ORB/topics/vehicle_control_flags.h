/**
 * @file vehicle_control_flags.h
 * Definition of the vehicle_control_flags ORB topic.
 * 
 * All control apps should depend their actions based on the flags set here.
 */

#ifndef TOPIC_VEHICLE_CONTROL_FLAGS_H_
#define TOPIC_VEHICLE_CONTROL_FLAGS_H_

	#include <stdint.h>
	#include "../ORB.h"
	#include "../../uav_library/common.h"

/**
	 * @addtogroup topics @{
	 */


	/**
	 * state of vehicle.
	 *
	 * Encodes the complete system state and is set by the commander app.
	 */
	struct vehicle_control_flags_s
	{
		bool_t flag_armed;
		bool_t flag_system_hil_enabled;				// XXX needs yet to be set by state machine helper

		bool_t flag_external_manual_override_ok;	/**< external override non-fatal for system. Only true for fixed wing */
		bool_t flag_control_manual_enabled;			/**< true if manual input is mixed in */
		bool_t flag_control_attitude_enabled;		/**< true if attitude stabilization is mixed in */
		bool_t flag_control_offboard_enabled;		/**< true if offboard control input is on */
		
		bool_t flag_control_rates_enabled;			/**< true if rates are stabilized */
		bool_t flag_control_velocity_enabled;		/**< true if horizontal velocity (implies direction) is controlled */
		bool_t flag_control_position_enabled;		/**< true if position is controlled */
		bool_t flag_control_altitude_enabled;		/**< true if altitude is controlled */
		bool_t flag_control_climb_rate_enabled;		/**< true if climb rate is controlled */

		bool_t flag_control_auto_enabled;	// TEMP
		uint8_t auto_state;					// TEMP navigation state for AUTO modes
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_control_flags);

#endif
