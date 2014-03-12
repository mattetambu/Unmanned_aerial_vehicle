/**
 * @file sensor_accel.h
 *
 * Definition of sensor_accel topic
 */

#ifndef TOPIC_SENSOR_ACCEL_H_
#define TOPIC_SENSOR_ACCEL_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * sensor_accel
	 */
	struct sensor_accel_s  {
		uint64_t error_count;
		
		float x;		/**< acceleration in the NED X board axis in m/s^2 */
		float y;		/**< acceleration in the NED Y board axis in m/s^2 */
		float z;		/**< acceleration in the NED Z board axis in m/s^2 */
		float temperature;	/**< temperature in degrees celsius */
		float range_m_s2;	/**< range in m/s^2 (+- this value) */
		float scaling;

		int16_t x_raw;
		int16_t y_raw;
		int16_t z_raw;
		int16_t temperature_raw;
	};

	/** accel scaling factors; Vout = Vscale * (Vin + Voffset) */
	struct accel_scale {
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
	ORB_DECLARE(sensor_accel);

#endif
