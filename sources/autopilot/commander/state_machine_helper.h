/**
 * @file state_machine_helper.h
 * State machine helper functions definitions
 */

#ifndef STATE_MACHINE_HELPER_H_
#define STATE_MACHINE_HELPER_H_

	#include "../../ORB/ORB.h"
	#include "../../ORB/topics/vehicle_status.h"
	#include "../../ORB/topics/safety.h"
	#include "../../ORB/topics/vehicle_control_flags.h"
	#include "../../ORB/topics/actuator/actuator_armed.h"
	#include "../../ORB/topics/position/vehicle_global_position.h"
	#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
	#include "../../uav_library/common.h"


	typedef enum {
		TRANSITION_DENIED = -1,
		TRANSITION_NOT_CHANGED = 0,
		TRANSITION_CHANGED
	} transition_result_t;


	/* function prototypes */
	bool_t is_safe(const struct vehicle_status_s *current_state, const struct safety_s *safety, const struct actuator_armed_s *armed);
	transition_result_t check_main_state_machine(struct manual_control_setpoint_s *sp_man, struct vehicle_status_s *current_status);
	transition_result_t check_navigation_state_machine(struct vehicle_status_s *status, struct vehicle_control_flags_s *control_flags, struct vehicle_global_position_s *global_pos);
	bool_t check_arming_state_changed();
	bool_t check_main_state_changed();
	bool_t check_navigation_state_changed();

	transition_result_t arming_state_transition(struct vehicle_status_s *current_state, const struct safety_s *safety,
	const struct vehicle_control_flags_s *control_flags, arming_state_t new_arming_state, struct actuator_armed_s *armed);
	transition_result_t main_state_transition(struct vehicle_status_s *current_state, main_flight_mode_t new_main_state, sub_flight_mode_t new_sub_flight_mode);
	transition_result_t navigation_state_transition(struct vehicle_status_s *status, navigation_state_t new_navigation_state, struct vehicle_control_flags_s *control_flags);
	transition_result_t hil_state_transition(hil_state_t new_state, int status_adv, struct vehicle_status_s *current_status, int control_flags_adv, struct vehicle_control_flags_s *current_control_flags, int armed_adv, struct actuator_armed_s *armed);
	void set_navigation_state_changed();


	/* global variables */
	extern bool_t arming_state_changed;
	extern bool_t main_state_changed;
	extern bool_t navigation_state_changed;
	extern float takeoff_alt;

#endif /* STATE_MACHINE_HELPER_H_ */
