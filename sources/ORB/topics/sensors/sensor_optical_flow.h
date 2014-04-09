/**
 * @file sensor_optical_flow.h
 * Definition of the optical flow uORB topic.
 */

#ifndef TOPIC_SENSOR_OPTICAL_FLOW_H_
#define TOPIC_SENSOR_OPTICAL_FLOW_H_

	#include <stdint.h>
	#include "../../ORB.h"

	/**
	 * @addtogroup topics
	 */

	/**
	 * Optical flow in NED body frame in SI units.
	 */
	struct sensor_optical_flow_s {
		uint64_t timestamp;
		int16_t flow_raw_x;		/**< flow in pixels in X direction, not rotation-compensated */
		int16_t flow_raw_y;		/**< flow in pixels in Y direction, not rotation-compensated */
		float flow_comp_x_m;		/**< speed over ground in meters, rotation-compensated */
		float flow_comp_y_m;		/**< speed over ground in meters, rotation-compensated */
		float ground_distance_m;	/**< Altitude / distance to ground in meters */
		uint8_t	quality;		/**< Quality of the measurement, 0: bad quality, 255: maximum quality */
		uint8_t sensor_id;		/**< id of the sensor emitting the flow value */

	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(sensor_optical_flow);

#endif
