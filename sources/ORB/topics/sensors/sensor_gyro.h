/**
 * @file sensor_gyro.h
 *
 * Definition of sensor_gyro topic
 */

#ifndef TOPIC_SENSOR_GYRO_H_
#define TOPIC_SENSOR_GYRO_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * sensor_gyro
	 */
	struct sensor_gyro_s {
		uint64_t error_count;
		
		float x;		/**< angular velocity in the NED X board axis in rad/s */
		float y;		/**< angular velocity in the NED Y board axis in rad/s */
		float z;		/**< angular velocity in the NED Z board axis in rad/s */
		float temperature;	/**< temperature in degrees celcius */
		float range_rad_s;
		float scaling;

		int16_t x_raw;
		int16_t y_raw;
		int16_t z_raw;
		int16_t temperature_raw;
	};

	/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */
	struct gyro_scale {
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
	ORB_DECLARE(sensor_gyro);

#endif
