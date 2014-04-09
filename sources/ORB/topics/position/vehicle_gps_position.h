/**
 * @file vehicle_gps_position.h
 * Definition of the GPS WGS84 uORB topic.
 */

#ifndef TOPIC_VEHICLE_GPS_H_
#define TOPIC_VEHICLE_GPS_H_
	
	#include <stdint.h>
	#include "../../ORB.h"
	#include "../../../uav_library/common.h"
	#include "../../../uav_library/time/drv_time.h"

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * GPS position in WGS84 coordinates.
	 */
	struct vehicle_gps_position_s
	{
		absolute_time timestamp_position;			/**< Timestamp for position information */
		int32_t latitude;					/**< Latitude in 1E7 degrees */
		int32_t longitude;					/**< Longitude in 1E7 degrees */
		int32_t altitude;					/**< Altitude in 1E3 meters (millimeters) above MSL  */

		absolute_time timestamp_variance;
		float s_variance_m_s;				/**< speed accuracy estimate m/s */
		float p_variance_m;				/**< position accuracy estimate m */
		float c_variance_rad;				/**< course accuracy estimate rad */
		uint8_t fix_type; 				/**< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.   */

		float eph_m;					/**< GPS HDOP horizontal dilution of position in m */
		float epv_m;					/**< GPS VDOP horizontal dilution of position in m */

		absolute_time timestamp_velocity;			/**< Timestamp for velocity informations */
		float vel_m_s;					/**< GPS ground speed (m/s) */
		float vel_n_m_s;				/**< GPS ground speed in m/s */
		float vel_e_m_s;				/**< GPS ground speed in m/s */
		float vel_d_m_s;				/**< GPS ground speed in m/s */
		float cog_rad;					/**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		bool_t vel_ned_valid;				/**< Flag to indicate if NED speed is valid */

		absolute_time timestamp_time;			/**< Timestamp for time information */
		absolute_time time_gps_usec;				/**< Timestamp (microseconds in GPS format), this is the timestamp which comes from the gps module   */

		absolute_time timestamp_satellites;			/**< Timestamp for sattelite information */
		uint8_t satellites_visible;			/**< Number of satellites visible. If unknown, set to 255 */
		uint8_t satellite_prn[20]; 			/**< Global satellite ID */
		uint8_t satellite_used[20];			/**< 0: Satellite not used, 1: used for localization */
		uint8_t satellite_elevation[20]; 		/**< Elevation (0: right on top of receiver, 90: on the horizon) of satellite */
		uint8_t satellite_azimuth[20];			/**< Direction of satellite, 0: 0 deg, 255: 360 deg. */
		uint8_t satellite_snr[20];			/**< Signal to noise ratio of satellite   */
		bool_t satellite_info_available;			/**< 0 for no info, 1 for info available */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_gps_position);

#endif
