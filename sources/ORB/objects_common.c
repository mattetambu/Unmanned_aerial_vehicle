/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file objects_common.cpp
 *
 * Common object definitions without a better home.
 */

/**
 * @defgroup topics List of all uORB topics.
 */

#include "ORB.h"
#include "../uav_library/common.h"


#include "topics/mission.h"
ORB_DEFINE(mission, struct mission_s);
ORB_DEFINE(mission_small, struct mission_small_s);

#include "topics/navigation_capabilities.h"
ORB_DEFINE(navigation_capabilities, struct navigation_capabilities_s);

#include "topics/parameter_update.h"
ORB_DEFINE_FREE_PUBLISH(parameter_update, struct parameter_update_s);

#include "topics/safety.h"
ORB_DEFINE(safety, struct safety_s);

#include "topics/subsystem_info.h"
ORB_DEFINE(subsystem_info, struct subsystem_info_s);

#include "topics/vehicle_attitude.h"
ORB_DEFINE(vehicle_attitude, struct vehicle_attitude_s);

#include "topics/vehicle_hil_attitude.h"
ORB_DEFINE(vehicle_hil_attitude, struct vehicle_hil_attitude_s);

#include "topics/vehicle_command.h"
ORB_DEFINE(vehicle_command, struct vehicle_command_s);

#include "topics/vehicle_control_flags.h"
ORB_DEFINE(vehicle_control_flags, struct vehicle_control_flags_s);

#include "topics/vehicle_status.h"
ORB_DEFINE(vehicle_status, struct vehicle_status_s);


/* actuator */
#include "topics/actuator/actuator_armed.h"
ORB_DEFINE(actuator_armed, struct actuator_armed_s);

#include "topics/actuator/actuator_controls.h"
ORB_DEFINE(actuator_controls, struct actuator_controls_s);

#include "topics/actuator/actuator_outputs.h"
ORB_DEFINE(actuator_outputs, struct actuator_outputs_s);

/* position */
#include "topics/position/home_position.h"
ORB_DEFINE(home_position, struct home_position_s);

#include "topics/position/takeoff_position.h"
ORB_DEFINE(takeoff_position, struct takeoff_position_s);

#include "topics/position/vehicle_global_position.h"
ORB_DEFINE(vehicle_global_position, struct vehicle_global_position_s);

#include "topics/position/vehicle_gps_position.h"
ORB_DEFINE(vehicle_gps_position, struct vehicle_gps_position_s);

#include "topics/position/vehicle_local_position.h"
ORB_DEFINE(vehicle_local_position, struct vehicle_local_position_s);

/* sensors */
#include "topics/sensors/battery_status.h"
ORB_DEFINE(battery_status, struct battery_status_s);

#include "topics/sensors/sensor_accel.h"
ORB_DEFINE(sensor_accel, struct sensor_accel_s);

#include "topics/sensors/sensor_airspeed.h"
ORB_DEFINE(sensor_airspeed, struct sensor_airspeed_s);

#include "topics/sensors/sensor_baro.h"
ORB_DEFINE(sensor_baro, struct sensor_baro_s);

#include "topics/sensors/sensor_combined.h"
ORB_DEFINE(sensor_combined, struct sensor_combined_s);

#include "topics/sensors/sensor_gyro.h"
ORB_DEFINE(sensor_gyro, struct sensor_gyro_s);

#include "topics/sensors/sensor_mag.h"
ORB_DEFINE(sensor_mag, struct sensor_mag_s);


/* setpoint */
#include "topics/setpoint/manual_control_setpoint.h"
ORB_DEFINE(manual_control_setpoint, struct manual_control_setpoint_s);

#include "topics/setpoint/vehicle_attitude_setpoint.h"
ORB_DEFINE(vehicle_attitude_setpoint, struct vehicle_attitude_setpoint_s);

#include "topics/setpoint/vehicle_global_position_set_triplet.h"
ORB_DEFINE(vehicle_global_position_set_triplet, struct vehicle_global_position_set_triplet_s);

#include "topics/setpoint/vehicle_global_position_setpoint.h"
ORB_DEFINE(vehicle_global_position_setpoint, struct vehicle_global_position_setpoint_s);

#include "topics/setpoint/vehicle_local_position_setpoint.h"
ORB_DEFINE(vehicle_local_position_setpoint, struct vehicle_local_position_setpoint_s);

#include "topics/setpoint/vehicle_rates_setpoint.h"
ORB_DEFINE(vehicle_rates_setpoint, struct vehicle_rates_setpoint_s);

#include "topics/setpoint/vehicle_speed_setpoint.h"
ORB_DEFINE(vehicle_speed_setpoint, struct vehicle_speed_setpoint_s);

