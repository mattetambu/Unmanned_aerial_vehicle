/*
 * @file multirotor_attitude_control.h
 *
 */

#ifndef MULTIROTOR_ATTITUDE_CONTROL_H_
#define MULTIROTOR_ATTITUDE_CONTROL_H_

	#include "../../uav_library/common.h"

	#include "../../ORB/ORB.h"
	#include "../../ORB/topics/vehicle_attitude.h"
	#include "../../ORB/topics/setpoint/vehicle_attitude_setpoint.h"
	#include "../../ORB/topics/setpoint/vehicle_rates_setpoint.h"
	#include "../../ORB/topics/actuator/actuator_controls.h"

	void multirotor_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp,
					 const struct vehicle_attitude_s *att, struct vehicle_rates_setpoint_s *rates_sp, bool_t control_yaw_position, bool_t reset_integral);

#endif /* MULTIROTOR_ATTITUDE_CONTROL_H_ */
