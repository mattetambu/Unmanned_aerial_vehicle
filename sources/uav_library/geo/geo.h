/**
 * @file geo.h
 *
 * Definition of geo / math functions to perform geodesic calculations
 */


#ifndef _LIBRARY_GEO_H_
#define _LIBRARY_GEO_H_

	#include <stdint.h>
	#include "../common.h"

	#define CONSTANTS_ONE_G							9.80665f		/* m/s^2		*/
	#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
	#define CONSTANTS_AIR_GAS_CONST					287.1f 			/* J/(kg * K)	*/
	#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* °C			*/
	#define CONSTANTS_RADIUS_OF_EARTH				6371000			/* meters (m)	*/

	struct crosstrack_error_s {
		bool_t past_end;	// Flag indicating we are past the end of the line/arc segment
		float distance;		// Distance in meters to closest point on line/arc
		float bearing;		// Bearing in radians to closest point on line/arc
	} ;


	/**
	 * Initializes the map transformation.
	 *
	 * Initializes the transformation between the geographic coordinate system and the azimuthal equidistant plane
	 * @param lat in degrees (47.1234567°, not 471234567°)
	 * @param lon in degrees (8.1234567°, not 81234567°)
	 */
	void map_projection_init(double lat_0, double lon_0);

	/**
	 * Transforms a point in the geographic coordinate system to the local azimuthal equidistant plane
	 * @param x north
	 * @param y east
	 * @param lat in degrees (47.1234567°, not 471234567°)
	 * @param lon in degrees (8.1234567°, not 81234567°)
	 */
	void map_projection_project(double lat, double lon, float *x, float *y);

	/**
	 * Transforms a point in the local azimuthal equidistant plane to the geographic coordinate system
	 *
	 * @param x north
	 * @param y east
	 * @param lat in degrees (47.1234567°, not 471234567°)
	 * @param lon in degrees (8.1234567°, not 81234567°)
	 */
	void map_projection_reproject(float x, float y, double *lat, double *lon);

	/**
	 * Returns the distance to the next waypoint in meters.
	 *
	 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
	 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
	 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
	 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
	 */
	float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

	/**
	 * Returns the bearing to the next waypoint in radians.
	 *
	 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
	 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
	 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
	 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
	 */
	float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

	void get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float* vx, float* vy);
	void get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next, float* vx, float* vy);
	int get_distance_to_line(struct crosstrack_error_s * crosstrack_error, double lat_now, double lon_now, double lat_start, double lon_start, double lat_end, double lon_end);
	int get_distance_to_arc(struct crosstrack_error_s * crosstrack_error, double lat_now, double lon_now, double lat_center, double lon_center,
	float radius, float arc_start_bearing, float arc_sweep);

	float _wrap_180(float bearing);
	float _wrap_360(float bearing);
	float _wrap_pi(float bearing);
	float _wrap_2pi(float bearing);


#endif
