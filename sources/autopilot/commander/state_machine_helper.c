/**
 * @file state_machine_helper.cpp
 * State machine helper functions implementations
 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#include "state_machine_helper.h"
#include "../../ORB/ORB.h"
#include "../../ORB/topics/vehicle_status.h"
#include "../../ORB/topics/vehicle_control_flags.h"
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/position/vehicle_global_position.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
#include "../../uav_library/common.h"


bool_t arming_state_changed = 1 /* true */;
bool_t main_state_changed = 1 /* true */;
bool_t navigation_state_changed = 1 /* true */;
float takeoff_alt = 5.0f;


transition_result_t
check_main_state_machine(struct manual_control_setpoint_s *sp_man, struct vehicle_status_s *current_status)
{
	/* evaluate the main state machine */
	transition_result_t res = TRANSITION_DENIED;

	switch (sp_man->mode_switch) {
		case mode_switch_manual:
			res = main_state_transition(current_status, main_state_manual, sub_flight_mode_manual);
			// TRANSITION_DENIED is not possible here
			break;

		case mode_switch_support:
			if (sp_man->second_switch == secondary_switch_easy) {
				res = main_state_transition(current_status, main_state_support, sub_flight_mode_easy);
				if (res != TRANSITION_DENIED)
					break;	// changed successfully or already in this state

				// else fallback to fly_by_wire
				fprintf (stderr, "Can't switch to easy flight mode\n");
			}

			if (sp_man->second_switch == secondary_switch_fly_by_wire) {
				res = main_state_transition(current_status, main_state_support, sub_flight_mode_fly_by_wire);
				if (res != TRANSITION_DENIED)
					break;	// changed successfully or already in this state

				// else fallback to stabilize
				fprintf (stderr, "Can't switch to fly_by_wire flight mode\n");
			}

			if (sp_man->second_switch == secondary_switch_stabilize || res == TRANSITION_DENIED) {
				res = main_state_transition(current_status, main_state_support, sub_flight_mode_stabilize);
				if (res != TRANSITION_DENIED)
					break;	// changed successfully or already in this state

				// else fallback to manual
				fprintf (stderr, "Can't switch to stabilize flight mode\n");
			}

			// fallback to manual
			res = main_state_transition(current_status, main_state_manual, sub_flight_mode_manual);
			// TRANSITION_DENIED is not possible here
			break;

		case mode_switch_auto:
			if (sp_man->second_switch == secondary_switch_rtl) {
				res = main_state_transition(current_status, main_state_auto, sub_flight_mode_rtl);
				if (res != TRANSITION_DENIED)
					break;	// changed successfully or already in this state

				// else fallback to fly_by_wire
				fprintf (stderr, "Can't switch to rtl flight mode\n");
			}
			else if (sp_man->second_switch == secondary_switch_loiter) {
				res = main_state_transition(current_status, main_state_auto, sub_flight_mode_loiter);
				if (res != TRANSITION_DENIED)
					break;	// changed successfully or already in this state

				// else fallback to fly_by_wire
				fprintf (stderr, "Can't switch to loiter flight mode\n");
			}
			else if (sp_man->second_switch == secondary_switch_mission) {
				res = main_state_transition(current_status, main_state_auto, sub_flight_mode_mission);
				if (res != TRANSITION_DENIED)
					break;	// changed successfully or already in this state

				// else fallback to fly_by_wire
				fprintf (stderr, "Can't switch to mission flight mode\n");
			}

			// else fallback to stabilize
			if (res == TRANSITION_DENIED) {
				res = main_state_transition(current_status, main_state_support, sub_flight_mode_stabilize);
				if (res != TRANSITION_DENIED)
					break;	// changed successfully or already in this state

				// else fallback to manual
				fprintf (stderr, "Can't switch to stabilize flight mode\n");
			}

			// fallback to manual
			res = main_state_transition(current_status, main_state_manual, sub_flight_mode_manual);
			// TRANSITION_DENIED is not possible here
			break;

		default:
			break;
	}

	return res;
}


transition_result_t
check_navigation_state_machine(struct vehicle_status_s *status, struct vehicle_control_flags_s *control_flags, struct vehicle_global_position_s *global_pos)
{
	transition_result_t res = TRANSITION_DENIED;

	if (status->main_flight_mode == main_state_auto) {
		if (status->arming_state == arming_state_armed || status->arming_state == arming_state_armed_error) {
			if (status->navigation_state == navigation_state_auto_land) {
				// TODO AUTO_LAND handling
			}
			if (status->navigation_state == navigation_state_auto_takeoff) {
				/* don't switch to other states until takeoff not completed */
				if ((global_pos->altitude - global_pos->ground_level) < takeoff_alt || status->condition_landed) {
					return TRANSITION_NOT_CHANGED;
				}
			}

			if (status->navigation_state != navigation_state_auto_takeoff &&
			    status->navigation_state != navigation_state_auto_loiter &&
			    status->navigation_state != navigation_state_auto_mission &&
			    status->navigation_state != navigation_state_auto_rtl) {
				/* possibly on ground, switch to TAKEOFF if needed */
				if ((global_pos->altitude - global_pos->ground_level) < takeoff_alt || status->condition_landed) {
					res = navigation_state_transition(status, navigation_state_auto_takeoff, control_flags);
					return res;
				}
			}

			/* switch to AUTO mode */
			if (status->rc_signal_found_once && !status->rc_signal_lost) {
				/* act depending on switches when manual control enabled */
				if (status->sub_flight_mode == sub_flight_mode_rtl) {
					/* RTL */
					res = navigation_state_transition(status, navigation_state_auto_rtl, control_flags);

				} else if (status->sub_flight_mode == sub_flight_mode_mission) {
					/* MISSION */
					res = navigation_state_transition(status, navigation_state_auto_mission, control_flags);

				} else {
					/* LOITER */
					res = navigation_state_transition(status, navigation_state_auto_loiter, control_flags);
				}
			} else {
				/* switch to MISSION when no RC control and first time in some AUTO mode  */
				if (status->navigation_state == navigation_state_auto_loiter ||
					status->navigation_state == navigation_state_auto_mission ||
					status->navigation_state == navigation_state_auto_rtl ||
					status->navigation_state == navigation_state_auto_land) {
					res = TRANSITION_NOT_CHANGED;

				} else {
					res = navigation_state_transition(status, navigation_state_auto_mission, control_flags);
				}
			}

		} else {
			/* disarmed, always switch to AUTO_READY */
			res = navigation_state_transition(status, navigation_state_auto_ready, control_flags);
		}

	} else /* manual control modes */ {
		if (status->rc_signal_lost && (status->arming_state == arming_state_armed || status->arming_state == arming_state_armed_error)) {
			/* switch to failsafe mode */
			bool_t manual_control_old = control_flags->flag_control_manual_enabled;

			if (!status->condition_landed) {
				/* in air: try to hold position */
				res = navigation_state_transition(status, navigation_state_vector, control_flags);

			} else {
				/* landed: don't try to hold position but land (if taking off) */
				res = TRANSITION_DENIED;
			}

			if (res == TRANSITION_DENIED) {
				res = navigation_state_transition(status, navigation_state_althold, control_flags);
			}

			control_flags->flag_control_manual_enabled = 0 /* false */;

			if (res == TRANSITION_NOT_CHANGED && manual_control_old) {
				/* mark navigation state as changed to force immediate flag publishing */
				set_navigation_state_changed();
				res = TRANSITION_CHANGED;
			}

			if (res == TRANSITION_CHANGED) {
				if (control_flags->flag_control_position_enabled) {
					//mavlink_log_critical(mavlink_fd, "#audio: FAILSAFE: POS HOLD");
					fprintf (stderr, "Failsafe - RC signal is lost so trying to hold position\n");
				} else {
					if (status->condition_landed) {
						//mavlink_log_critical(mavlink_fd, "#audio: FAILSAFE: ALT HOLD (LAND)");
						fprintf (stderr, "Failsafe - RC signal is lost so trying to hold altitude (LAND)\n");
					} else {
						//mavlink_log_critical(mavlink_fd, "#audio: FAILSAFE: ALT HOLD");
						fprintf (stderr, "Failsafe - RC signal is lost so trying to hold altitude\n");
					}
				}
			}

		} else {
			switch (status->main_flight_mode) {
				case main_state_manual:
					res = navigation_state_transition(status, status->is_rotary_wing ? navigation_state_stabilize : navigation_state_direct, control_flags);
					break;

				case main_state_support:
					if (status->sub_flight_mode == sub_flight_mode_fly_by_wire)
						res = navigation_state_transition(status, navigation_state_vector, control_flags);
					else
						res = navigation_state_transition(status, navigation_state_althold, control_flags);

					break;

				default:
					break;
			}
		}
	}

	return res;
}



transition_result_t
arming_state_transition(struct vehicle_status_s *status, const struct safety_s *safety,
	const struct vehicle_control_flags_s *control_flags,
	arming_state_t new_arming_state, struct actuator_armed_s *armed)
{
	/*
	 * Perform an atomic state update
	 */
	//irqstate_t flags = irqsave();

	transition_result_t ret = TRANSITION_DENIED;

	/* only check transition if the new state is actually different from the current one */
	if (new_arming_state == status->arming_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		/* enforce lockdown in HIL */
		if (control_flags->flag_system_hil_enabled) {
			armed->lockdown = 1 /* true */;
		} else {
			armed->lockdown = 0 /* false */;
		}

		switch (new_arming_state) {
			case arming_state_init:

				/* allow going back from INIT for calibration */
				if (status->arming_state == arming_state_standby) {
					ret = TRANSITION_CHANGED;
					armed->armed = 0 /* false */;
					armed->ready_to_arm = 0 /* false */;
				}

				break;

			case arming_state_standby:

				/* allow coming from INIT and disarming from ARMED */
				if (status->arming_state == arming_state_init
					|| status->arming_state == arming_state_armed
					|| control_flags->flag_system_hil_enabled) {

					/* sensors need to be initialized for STANDBY state */
					if (status->condition_system_sensors_initialized) {
						ret = TRANSITION_CHANGED;
						armed->armed = 0 /* false */;
						armed->ready_to_arm = 1 /* true */;
					}
				}

				break;

			case arming_state_armed:

				/* allow arming from STANDBY and IN-AIR-RESTORE */
				if ((status->arming_state == arming_state_standby
					 || status->arming_state == arming_state_in_air_restore)
					&& (!safety->safety_switch_available || safety->safety_off || control_flags->flag_system_hil_enabled)) { /* only allow arming if safety is off */
					ret = TRANSITION_CHANGED;
					armed->armed = 1 /* true */;
					armed->ready_to_arm = 1 /* true */;
				}

				break;

			case arming_state_armed_error:

				/* an armed error happens when ARMED obviously */
				if (status->arming_state == arming_state_armed) {
					ret = TRANSITION_CHANGED;
					armed->armed = 1 /* true */;
					armed->ready_to_arm = 0 /* false */;
				}

				break;

			case arming_state_standby_error:

				/* a disarmed error happens when in STANDBY or in INIT or after ARMED_ERROR */
				if (status->arming_state == arming_state_standby
					|| status->arming_state == arming_state_init
					|| status->arming_state == arming_state_armed_error) {
					ret = TRANSITION_CHANGED;
					armed->armed = 0 /* false */;
					armed->ready_to_arm = 0 /* false */;
				}

				break;

			case arming_state_reboot:

				/* an armed error happens when ARMED obviously */
				if (status->arming_state == arming_state_init
					|| status->arming_state == arming_state_standby
					|| status->arming_state == arming_state_standby_error) {
					ret = TRANSITION_CHANGED;
					armed->armed = 0 /* false */;
					armed->ready_to_arm = 0 /* false */;
				}

				break;

			case arming_state_in_air_restore:

				/* XXX implement */
				break;

			default:
				break;
		}

		if (ret == TRANSITION_CHANGED) {
			status->arming_state = new_arming_state;
			arming_state_changed = 1 /* true */;
		}
	}

	/* end of atomic state update */
	//irqrestore(flags);

	if (ret == TRANSITION_DENIED)
		fprintf (stderr, "arming transition rejected");

	return ret;
}

bool_t is_safe(const struct vehicle_status_s *status, const struct safety_s *safety, const struct actuator_armed_s *armed)
{
	// System is safe if:
	// 1) Not armed
	// 2) Armed, but in software lockdown (HIL)
	// 3) Safety switch is present AND engaged -> actuators locked
	if (!armed->armed || (armed->armed && armed->lockdown) || (safety->safety_switch_available && !safety->safety_off)) {
		return 1 /* true */;

	} else {
		return 0 /* false */;
	}
}

bool_t
check_arming_state_changed()
{
	if (arming_state_changed) {
		arming_state_changed = 0 /* false */;
		return 1 /* true */;

	} else {
		return 0 /* false */;
	}
}

transition_result_t
main_state_transition(struct vehicle_status_s *current_state, main_flight_mode_t new_main_state, sub_flight_mode_t new_sub_flight_mode)
{
	transition_result_t ret = TRANSITION_DENIED;

	/* only check transition if the new state is actually different from the current one */
	if (new_main_state == current_state->main_flight_mode) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		switch (new_main_state) {
			case main_state_manual:
				ret = TRANSITION_CHANGED;
				break;

			case main_state_support:

				/* need at minimum altitude estimate */
				if (new_sub_flight_mode == sub_flight_mode_stabilize &&
					(current_state->condition_local_altitude_valid ||
					current_state->condition_global_position_valid)) {
					ret = TRANSITION_CHANGED;
				}

				/* need at minimum altitude estimate */
				if (new_sub_flight_mode == sub_flight_mode_fly_by_wire &&
					(current_state->condition_local_altitude_valid ||
					current_state->condition_global_position_valid)) {
					ret = TRANSITION_CHANGED;
				}

				/* need at minimum altitude estimate */
				if (new_sub_flight_mode == sub_flight_mode_easy &&
					(current_state->condition_local_position_valid ||
					current_state->condition_global_position_valid)) {
					ret = TRANSITION_CHANGED;
				}

				break;

			case main_state_auto:

				/* need global position estimate */
				if (current_state->condition_global_position_valid) {
					ret = TRANSITION_CHANGED;
				}

				break;
		}

		if (ret == TRANSITION_CHANGED) {
			current_state->main_flight_mode = new_main_state;
			current_state->sub_flight_mode = new_sub_flight_mode;
			main_state_changed = 1 /* true */;
		}
	}

	return ret;
}

bool_t
check_main_state_changed()
{
	if (main_state_changed) {
		main_state_changed = 0 /* false */;
		return 1 /* true */;

	} else {
		return 0 /* false */;
	}
}

transition_result_t
navigation_state_transition(struct vehicle_status_s *status, navigation_state_t new_navigation_state, struct vehicle_control_flags_s *control_flags)
{
	transition_result_t ret = TRANSITION_DENIED;

	/* only check transition if the new state is actually different from the current one */
	if (new_navigation_state == status->navigation_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		switch (new_navigation_state) {
		case navigation_state_direct:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 1 /* true */;
			control_flags->flag_control_attitude_enabled = 0 /* false */;
			control_flags->flag_control_velocity_enabled = 0 /* false */;
			control_flags->flag_control_position_enabled = 0 /* false */;
			control_flags->flag_control_altitude_enabled = 0 /* false */;
			control_flags->flag_control_climb_rate_enabled = 0 /* false */;
			control_flags->flag_control_manual_enabled = 1 /* true */;
			control_flags->flag_control_auto_enabled = 0 /* false */;
			break;

		case navigation_state_stabilize:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 1 /* true */;
			control_flags->flag_control_attitude_enabled = 1 /* true */;
			control_flags->flag_control_velocity_enabled = 0 /* false */;
			control_flags->flag_control_position_enabled = 0 /* false */;
			control_flags->flag_control_altitude_enabled = 0 /* false */;
			control_flags->flag_control_climb_rate_enabled = 0 /* false */;
			control_flags->flag_control_manual_enabled = 1 /* true */;
			control_flags->flag_control_auto_enabled = 0 /* false */;
			break;

		case navigation_state_althold:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 1 /* true */;
			control_flags->flag_control_attitude_enabled = 1 /* true */;
			control_flags->flag_control_velocity_enabled = 0 /* false */;
			control_flags->flag_control_position_enabled = 0 /* false */;
			control_flags->flag_control_altitude_enabled = 1 /* true */;
			control_flags->flag_control_climb_rate_enabled = 1 /* true */;
			control_flags->flag_control_manual_enabled = 1 /* true */;
			control_flags->flag_control_auto_enabled = 0 /* false */;
			break;

		case navigation_state_vector:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 1 /* true */;
			control_flags->flag_control_attitude_enabled = 1 /* true */;
			control_flags->flag_control_velocity_enabled = 1 /* true */;
			control_flags->flag_control_position_enabled = 1 /* true */;
			control_flags->flag_control_altitude_enabled = 1 /* true */;
			control_flags->flag_control_climb_rate_enabled = 1 /* true */;
			control_flags->flag_control_manual_enabled = 1 /* true */;
			control_flags->flag_control_auto_enabled = 0 /* false */;
			break;

		case navigation_state_auto_ready:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 0 /* false */;
			control_flags->flag_control_attitude_enabled = 0 /* false */;
			control_flags->flag_control_velocity_enabled = 0 /* false */;
			control_flags->flag_control_position_enabled = 0 /* false */;
			control_flags->flag_control_altitude_enabled = 0 /* false */;
			control_flags->flag_control_climb_rate_enabled = 0 /* false */;
			control_flags->flag_control_manual_enabled = 0 /* false */;
			control_flags->flag_control_auto_enabled = 1 /* true */;
			break;

		case navigation_state_auto_takeoff:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 1 /* true */;
			control_flags->flag_control_attitude_enabled = 1 /* true */;
			control_flags->flag_control_velocity_enabled = 1 /* true */;
			control_flags->flag_control_position_enabled = 1 /* true */;
			control_flags->flag_control_altitude_enabled = 1 /* true */;
			control_flags->flag_control_climb_rate_enabled = 1 /* true */;
			control_flags->flag_control_manual_enabled = 0 /* false */;
			control_flags->flag_control_auto_enabled = 1 /* true */;
			break;

		case navigation_state_auto_loiter:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 1 /* true */;
			control_flags->flag_control_attitude_enabled = 1 /* true */;
			control_flags->flag_control_velocity_enabled = 1 /* true */;
			control_flags->flag_control_position_enabled = 1 /* true */;
			control_flags->flag_control_altitude_enabled = 1 /* true */;
			control_flags->flag_control_climb_rate_enabled = 1 /* true */;
			control_flags->flag_control_manual_enabled = 0 /* false */;
			control_flags->flag_control_auto_enabled = 0 /* false */;
			break;

		case navigation_state_auto_mission:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 1 /* true */;
			control_flags->flag_control_attitude_enabled = 1 /* true */;
			control_flags->flag_control_velocity_enabled = 1 /* true */;
			control_flags->flag_control_position_enabled = 1 /* true */;
			control_flags->flag_control_altitude_enabled = 1 /* true */;
			control_flags->flag_control_climb_rate_enabled = 1 /* true */;
			control_flags->flag_control_manual_enabled = 0 /* false */;
			control_flags->flag_control_auto_enabled = 1 /* true */;
			break;

		case navigation_state_auto_rtl:
			ret = TRANSITION_CHANGED;
			control_flags->flag_control_rates_enabled = 1 /* true */;
			control_flags->flag_control_attitude_enabled = 1 /* true */;
			control_flags->flag_control_velocity_enabled = 1 /* true */;
			control_flags->flag_control_position_enabled = 1 /* true */;
			control_flags->flag_control_altitude_enabled = 1 /* true */;
			control_flags->flag_control_climb_rate_enabled = 1 /* true */;
			control_flags->flag_control_manual_enabled = 0 /* false */;
			control_flags->flag_control_auto_enabled = 1 /* true */;
			break;

		case navigation_state_auto_land:

			/* deny transitions from landed state */
			if (status->navigation_state != navigation_state_auto_ready) {
				ret = TRANSITION_CHANGED;
				control_flags->flag_control_rates_enabled = 1 /* true */;
				control_flags->flag_control_attitude_enabled = 1 /* true */;
				control_flags->flag_control_velocity_enabled = 1 /* true */;
				control_flags->flag_control_position_enabled = 1 /* true */;
				control_flags->flag_control_altitude_enabled = 1 /* true */;
				control_flags->flag_control_climb_rate_enabled = 1 /* true */;
				control_flags->flag_control_manual_enabled = 0 /* false */;
				control_flags->flag_control_auto_enabled = 1 /* true */;
			}

			break;

		default:
			break;
		}

		if (ret == TRANSITION_CHANGED) {
			status->navigation_state = new_navigation_state;
			control_flags->auto_state = status->navigation_state;
			navigation_state_changed = 1 /* true */;
		}
	}

	return ret;
}

bool_t
check_navigation_state_changed()
{
	if (navigation_state_changed) {
		navigation_state_changed = 0 /* false */;
		return 1 /* true */;

	} else {
		return 0 /* false */;
	}
}

void
set_navigation_state_changed()
{
	navigation_state_changed = 1 /* true */;
}

/**
* Transition from one hil state to another
*/
transition_result_t hil_state_transition(hil_state_t new_state, int status_adv, struct vehicle_status_s *current_status, int control_flags_adv, struct vehicle_control_flags_s *current_control_flags, int armed_adv, struct actuator_armed_s *armed/*, const int mavlink_fd*/)
{
	transition_result_t ret = TRANSITION_DENIED;

	//fprintf (stderr, "Current state: %d, requested state: %d", current_status->hil_state, new_state);

	if (current_status->hil_state == new_state) {
		fprintf (stderr, "Hil state not changed");
		ret = TRANSITION_NOT_CHANGED;

	} else {

		switch (new_state) {

		case hil_state_off:

			/* we're in HIL and unexpected things can happen if we disable HIL now */
			//mavlink_log_critical(mavlink_fd, "Not switching off HIL (safety)");
			break;

		case hil_state_on:

			if (current_status->arming_state == arming_state_init
			    || current_status->arming_state == arming_state_standby
			    || current_status->arming_state == arming_state_standby_error) {

				//mavlink_log_critical(mavlink_fd, "Switched to ON hil state");
				ret = TRANSITION_CHANGED;
			}

			break;

		default:
			fprintf (stderr, "Unknown hil state");
			break;
		}
	}

	if (ret == TRANSITION_CHANGED) {
		current_status->hil_state = new_state;
		current_control_flags->flag_system_hil_enabled = 1 /* true */;

		/* enforce lockdown in HIL */
		armed->lockdown = 1 /* true */;

		orb_publish(ORB_ID(vehicle_status), status_adv, current_status);
		orb_publish(ORB_ID(vehicle_control_flags), control_flags_adv, current_control_flags);
		orb_publish(ORB_ID(actuator_armed), armed_adv, &armed);

	} else {
		//mavlink_log_critical(mavlink_fd, "REJECTING invalid hil state transition");
	}

	return ret;
}
