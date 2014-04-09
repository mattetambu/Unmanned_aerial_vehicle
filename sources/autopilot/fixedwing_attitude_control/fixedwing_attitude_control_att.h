/*
 * @file Fixed Wing Attitude Control
 */

#ifndef FIXEDWING_ATT_CONTROL_ATT_H_
#define FIXEDWING_ATT_CONTROL_ATT_H_

	#include "../../ORB/topics/vehicle_attitude.h"
	#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
	#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
	#include "../../ORB/topics/position/vehicle_global_position.h"

	int fixedwing_attitude_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
				   const float speed_body[], struct vehicle_rates_setpoint_s *rates_sp);

#endif /* FIXEDWING_ATT_CONTROL_ATT_H_ */
