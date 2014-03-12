/**
 * @file sensor_combined.h
 * Definition of the sensor_combined uORB topic.
 */

#ifndef TOPIC_SENSOR_COMBINED_H_
#define TOPIC_SENSOR_COMBINED_H_

	#include <stdint.h>
	#include "../../ORB.h"
	#include "../../../uav_library/time/drv_time.h"


	enum MAGNETOMETER_MODE {
		MAGNETOMETER_MODE_NORMAL = 0,
		MAGNETOMETER_MODE_POSITIVE_BIAS,
		MAGNETOMETER_MODE_NEGATIVE_BIAS
	};

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Sensor readings in raw and SI-unit form.
	 *
	 * These values are read from the sensors. Raw values are in sensor-specific units,
	 * the scaled values are in SI-units, as visible from the ending of the variable
	 * or the comments. The use of the SI fields is in general advised, as these fields
	 * are scaled and offset-compensated where possible and do not change with board
	 * revisions and sensor updates.
	 *
	 */
	struct sensor_combined_s {
		absolute_time timestamp;
		
		int16_t	gyro_raw[3];			/**< Raw sensor values of angular velocity        */
		uint16_t gyro_counter;			/**< Number of raw measurments taken              */
		float gyro_rad_s[3];			/**< Angular velocity in radian per seconds       */
		
		int16_t accelerometer_raw[3];		/**< Raw acceleration in NED body frame           */
		uint32_t accelerometer_counter;		/**< Number of raw acc measurements taken         */
		float accelerometer_m_s2[3];		/**< Acceleration in NED body frame, in m/s^2     */
		int accelerometer_mode;			/**< Accelerometer measurement mode */
		float accelerometer_range_m_s2;		/**< Accelerometer measurement range in m/s^2 */

		int16_t	magnetometer_raw[3];		/**< Raw magnetic field in NED body frame         */
		float magnetometer_ga[3];		/**< Magnetic field in NED body frame, in Gauss   */
		int magnetometer_mode;			/**< Magnetometer measurement mode */
		float magnetometer_range_ga;		/**< ± measurement range in Gauss */
		float magnetometer_cuttoff_freq_hz;	/**< Internal analog low pass frequency of sensor */
		uint32_t magnetometer_counter;		/**< Number of raw mag measurements taken         */
		
		float baro_pres_mbar;			/**< Barometric pressure, already temp. comp.     */
		float baro_alt_meter;			/**< Altitude, already temp. comp.                */
		float baro_temp_celcius;		/**< Temperature in degrees celsius               */
		float adc_voltage_v[4];			/**< ADC voltages of ADC Chan 10/11/12/13 or -1      */
		float mcu_temp_celcius;			/**< Internal temperature measurement of MCU */
		uint32_t baro_counter;			/**< Number of raw baro measurements taken        */

		float differential_pressure_pa;				/**< Airspeed sensor differential pressure                  */ 
		uint32_t differential_pressure_counter;		/**< Number of raw differential pressure measurements taken */
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(sensor_combined);

#endif
