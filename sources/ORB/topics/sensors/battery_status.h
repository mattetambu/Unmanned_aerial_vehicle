/**
 * @file battery_status.h
 *
 * Definition of the battery status ORB topic.
 */

#ifndef TOPIC_BATTERY_STATUS_H_
#define TOPIC_BATTERY_STATUS_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Battery voltages and status
	 */
	struct battery_status_s {
		float   	voltage_v;		/**< Battery voltage in volts, filtered           	 */
		float		current_a;		/**< Battery current in amperes, filtered, -1 if unknown */
		float		discharged_mah;		/**< Discharged amount in mAh, filtered, -1 if unknown	 */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(battery_status);

#endif
