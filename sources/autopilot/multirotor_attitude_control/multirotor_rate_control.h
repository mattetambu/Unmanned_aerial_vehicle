/*
 * @file multirotor_attitude_control.h
 *
 * Definition of rate controller for multirotors.
 *
 */

#ifndef MULTIROTOR_RATE_CONTROL_H_
#define MULTIROTOR_RATE_CONTROL_H_

	#include "../../uav_library/common.h"

	#include "../../ORB/ORB.h"
	#include "../../ORB/topics/vehicle_attitude.h"
	#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
	#include "../../ORB/topics/actuator/actuator_controls.h"


	void multirotor_control_rates(const struct vehicle_rates_setpoint_s *rate_sp,
					  const float rates[], struct actuator_controls_s *actuators, bool_t reset_integral);

#endif /* MULTIROTOR_RATE_CONTROL_H_ */
