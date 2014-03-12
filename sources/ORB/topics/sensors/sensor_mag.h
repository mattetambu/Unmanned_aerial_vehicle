/**
 * @file sensor_mag.h
 *
 * Definition of sensor_mag topic
 */

#ifndef TOPIC_SENSOR_MAG_H_
#define TOPIC_SENSOR_MAG_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * sensor_mag
	 * Output values are in gauss.
	 */
	struct sensor_mag_s {
		uint64_t error_count;
		
		float x;
		float y;
		float z;
		float range_ga;
		float scaling;

		int16_t x_raw;
		int16_t y_raw;
		int16_t z_raw;
	};

	/** mag scaling factors; Vout = (Vin * Vscale) + Voffset */
	struct mag_scale {
		float	x_offset;
		float	x_scale;
		float	y_offset;
		float	y_scale;
		float	z_offset;
		float	z_scale;
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(sensor_mag);

#endif
