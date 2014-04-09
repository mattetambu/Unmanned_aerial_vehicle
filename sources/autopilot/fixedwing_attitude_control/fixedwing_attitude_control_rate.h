/*
 * @file Fixed Wing Attitude Rate Control
 */

#ifndef FIXEDWING_ATT_CONTROL_RATE_H_
#define FIXEDWING_ATT_CONTROL_RATE_H_

	#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
	#include "../../ORB/topics/actuator/actuator_controls.h"

	int fixedwing_attitude_control_rates(const struct vehicle_rates_setpoint_s *rate_sp, const float rates[], struct actuator_controls_s *actuators);

#endif /* FIXEDWING_ATT_CONTROL_RATE_H_ */
