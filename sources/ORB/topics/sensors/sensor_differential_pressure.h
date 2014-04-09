/**
 * @file sensor_differential_pressure.h
 *
 * Definition of differential pressure topic
 */

#ifndef TOPIC_SENSOR_DIFFERENTIAL_PRESSURE_H_
#define TOPIC_SENSOR_DIFFERENTIAL_PRESSURE_H_

	#include "../../ORB.h"
	#include <stdint.h>

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Differential pressure.
	 */
	struct sensor_differential_pressure_s {
		uint64_t	error_count;
		uint16_t	differential_pressure_pa;	/**< Differential pressure reading */
		uint16_t	max_differential_pressure_pa;	/**< Maximum differential pressure reading */
		float		voltage;			/**< Voltage from analog airspeed sensors (voltage divider already compensated) */
		float		temperature;			/**< Temperature provided by sensor */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(sensor_differential_pressure);

#endif
