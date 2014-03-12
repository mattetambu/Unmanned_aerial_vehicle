/**
 * @file sensor_baro.h
 *
 * Definition of sensor_baro topic
 */

#ifndef TOPIC_SENSOR_BARO_H_
#define TOPIC_SENSOR_BARO_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * sensor_baro
	 */
	struct sensor_baro_s {
		uint64_t error_count;
		
		float pressure;
		float altitude;
		float temperature;
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(sensor_baro);

#endif
