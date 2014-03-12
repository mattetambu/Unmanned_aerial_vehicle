/**
 * @file commander_main.c
 *
 * Main system state machine implementation.
 *
 */

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <string.h>
#include <math.h>
#include <poll.h>

#include "../../ORB/ORB.h"
#include "../../ORB/topics/sensors/battery_status.h"
#include "../../ORB/topics/safety.h"
#include "../../ORB/topics/setpoint/manual_control_setpoint.h"
#include "../../ORB/topics/vehicle_status.h"
#include "../../ORB/topics/vehicle_control_flags.h"
#include "../../ORB/topics/position/home_position.h"
#include "../../ORB/topics/position/takeoff_position.h"
#include "../../ORB/topics/position/vehicle_global_position.h"
#include "../../ORB/topics/actuator/actuator_controls.h"
#include "../../ORB/topics/actuator/actuator_armed.h"
#include "../../ORB/topics/vehicle_command.h"
#include "../../ORB/topics/subsystem_info.h"
#include "../../ORB/topics/parameter_update.h"

#include "commander_main.h"
#include "state_machine_helper.h"
#include "../../uav_library/common.h"
#include "../../uav_library/param/param.h"
#include "../../uav_library/time/drv_time.h"


#define ALWAYS_CONSIDER_MAN_SP_AS_VALID	// XXX TO REMOVE
#define LOW_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS 1000.0f
#define CRITICAL_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS 100.0f

/* Decouple update interval and hysteris counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL		25000	// us
#define COMMANDER_ERROR_OUTPUT_INTERVAL		2500	// us
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))
#define LOW_VOLTAGE_BATTERY_COUNTER_LIMIT (LOW_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)
#define CRITICAL_VOLTAGE_BATTERY_COUNTER_LIMIT (CRITICAL_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define STICK_THRUST_RANGE 1.0f
#define POSITION_TIMEOUT 1000000 /**< consider the local or global position estimate invalid after 1s */
#define RC_TIMEOUT 200000 /**< consider the manaul control setpoint invalid after 0.2s */
#define	WANT_TO_ARM_TIME_COUNTER_LIMIT	2000000
#define BATTERY_INFO_EXPORT_TIME	20000000

enum MAV_MODE_FLAG {
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, /* 0b00000001 Reserved for future use. | */
	MAV_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	MAV_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	MAV_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	MAV_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	MAV_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */
	MAV_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
	MAV_MODE_FLAG_ENUM_END = 129, /*  | */
};

/* Mavlink file descriptors */
//static int mavlink_fd;

/* initialize */
bool_t commander_initialized = 0 /* false */;
bool_t battery_initialized = 0 /* false */;
unsigned int battery_counter = 0;
param_t bat_volt_empty, bat_volt_full, bat_n_cells;

/* advert */
orb_advert_t control_flags_adv = -1;
orb_advert_t status_adv = -1;
orb_advert_t armed_adv = -1;


int
commander_params_define ()
{
	PARAM_DEFINE_FLOAT (BAT_N_CELLS, 4.0f);
	PARAM_DEFINE_FLOAT (BAT_V_EMPTY, 3.2f);
	PARAM_DEFINE_FLOAT (BAT_V_FULL, 4.05f);

	PARAM_DEFINE_FLOAT (TAKEOFF_ALT, 7.5f); // XXX check this value (default 7.5f)
	PARAM_DEFINE_FLOAT (LAND_ALT, 2.0f); // XXX check this value
	PARAM_DEFINE_FLOAT (LAND_TIME, 3.0f);
	PARAM_DEFINE_FLOAT (LAND_THRUST, 0.25f);

	return 0;
}

void
check_valid(absolute_time timestamp, absolute_time timeout, bool_t valid_in, bool_t *valid_out, bool_t *changed)
{
	absolute_time t = get_absolute_time();
	bool_t valid_new = (t < timestamp + timeout && t > timeout && valid_in);

	if (*valid_out != valid_new) {
		*valid_out = valid_new;
		*changed = 1 /* true */;
	}
}

int
sensors_calibration (struct vehicle_command_s *cmd)
{
	if (!cmd)
	{
		/*if (!do_gyro_calibration(mavlink_fd) &&
			!do_mag_calibration(mavlink_fd) &&
			!do_press_calibration(mavlink_fd) &&
			!do_rc_calibration(mavlink_fd) &&
			!do_accel_calibration(mavlink_fd) &&
			!do_airspeed_calibration(mavlink_fd))
			calib_ret = 0;*/
	}
	else {
		// if ((int)(cmd->param1) == 1) {
			// /* gyro calibration */
			// answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
			// calib_ret = do_gyro_calibration(mavlink_fd);

		// } else if ((int)(cmd->param2) == 1) {
			// /* magnetometer calibration */
			// answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
			// calib_ret = do_mag_calibration(mavlink_fd);

		// } else if ((int)(cmd->param3) == 1) {
			// /* zero-altitude pressure calibration */
			// answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);

		// } else if ((int)(cmd->param4) == 1) {
			// /* RC calibration */
			// answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
			// calib_ret = do_rc_calibration(mavlink_fd);

		// } else if ((int)(cmd->param5) == 1) {
			// /* accelerometer calibration */
			// answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
			// calib_ret = do_accel_calibration(mavlink_fd);

		// } else if ((int)(cmd->param6) == 1) {
			// /* airspeed calibration */
			// answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
			// calib_ret = do_airspeed_calibration(mavlink_fd);
		// }
	}

	return 0;
}

/**
 * React to commands that are sent e.g. from the mavlink module.
 */
uint8_t handle_command(struct vehicle_status_s *status, const struct safety_s *safety, struct vehicle_control_flags_s *control_flags, struct vehicle_command_s *cmd, struct actuator_armed_s *armed)
{
	/* result of the command */
	uint8_t result = VEHICLE_CMD_RESULT_UNSUPPORTED;
	uint8_t base_mode;
	uint8_t custom_main_mode;
	transition_result_t arming_res, main_res, nav_res, hil_res;

	/* only handle high-priority commands here */

	/* request to set different system mode */
	switch (cmd->command) {
		case VEHICLE_CMD_DO_SET_MODE:
			base_mode = (uint8_t) cmd->param1;
			custom_main_mode = (uint8_t) cmd->param2;
			arming_res = TRANSITION_NOT_CHANGED;

			/* set HIL state */
			hil_state_t new_hil_state = (base_mode & MAV_MODE_FLAG_HIL_ENABLED) ? hil_state_on : hil_state_off;
			hil_res = hil_state_transition(new_hil_state, status_adv, status, control_flags_adv, control_flags, armed_adv, armed/*, mavlink_fd*/);

			/* if HIL got enabled, reset battery status state */
			if (hil_res != TRANSITION_DENIED /* OK */ && control_flags->flag_system_hil_enabled) {
				/* reset the arming mode to disarmed */
				arming_res = arming_state_transition(status, safety, control_flags, arming_state_standby, armed);

				if (arming_res != TRANSITION_DENIED) {
					//mavlink_log_info(mavlink_fd, "[cmd] HIL: Reset ARMED state to standby");

				} else {
					//mavlink_log_info(mavlink_fd, "[cmd] HIL: FAILED resetting armed state");
				}
			}

			/* set arming state */
			arming_res = TRANSITION_NOT_CHANGED;

			if (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
				if ((safety->safety_switch_available && !safety->safety_off) && !control_flags->flag_system_hil_enabled) {
					fprintf (stderr, "WARNING: Not arming, press safety switch first.\n");
					arming_res = TRANSITION_DENIED;

				} else {
					arming_res = arming_state_transition(status, safety, control_flags, arming_state_armed, armed);
				}

				if (arming_res == TRANSITION_CHANGED) {
					//mavlink_log_info(mavlink_fd, "[cmd] ARMED by command");
				}

			} else {
				if (status->arming_state == arming_state_armed || status->arming_state == arming_state_armed_error) {
					arming_state_t new_arming_state = (status->arming_state == arming_state_armed ? arming_state_standby : arming_state_standby_error);
					arming_res = arming_state_transition(status, safety, control_flags, new_arming_state, armed);

					if (arming_res == TRANSITION_CHANGED) {
						//mavlink_log_info(mavlink_fd, "[cmd] DISARMED by command");
					}

				} else {
					arming_res = TRANSITION_NOT_CHANGED;
				}
			}

			/* set main state */
			main_res = TRANSITION_DENIED;

			if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == CUSTOM_MAIN_MODE_MANUAL) {
					/* MANUAL */
					main_res = main_state_transition(status, main_state_manual, sub_flight_mode_manual);

				} else if (custom_main_mode == CUSTOM_MAIN_MODE_SUPPORT) {
					/* STABILIZE */
					main_res = main_state_transition(status, main_state_support, sub_flight_mode_stabilize);

				} else if (custom_main_mode == CUSTOM_MAIN_MODE_AUTO) {
					/* MISSION */
					main_res = main_state_transition(status, main_state_auto, sub_flight_mode_mission);
				}

			} else {
				/* use base mode */
				if (base_mode & MAV_MODE_FLAG_AUTO_ENABLED) {
					/* AUTO */
					main_res = main_state_transition(status, main_state_auto, sub_flight_mode_mission);

				} else if (base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) {
						/* EASY */
						main_res = main_state_transition(status, main_state_support, sub_flight_mode_fly_by_wire);
					}
					else if (base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) {
						/* STABILIZE */
						main_res = main_state_transition(status, main_state_support, sub_flight_mode_stabilize);
					}
					else {
						/* MANUAL */
						main_res = main_state_transition(status, main_state_manual, sub_flight_mode_manual);
					}
				}
			}

			if (arming_res != TRANSITION_DENIED && main_res != TRANSITION_DENIED) {
				result = VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}

			break;

		case VEHICLE_CMD_NAV_TAKEOFF:
			if (armed->armed) {
				nav_res = navigation_state_transition(status, navigation_state_auto_takeoff, control_flags);

				if (nav_res == TRANSITION_CHANGED) {
					//mavlink_log_info(mavlink_fd, "[cmd] TAKEOFF on command");
				}

				if (nav_res != TRANSITION_DENIED) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

			} else {
				/* reject TAKEOFF not armed */
				result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}

			break;

		case VEHICLE_CMD_COMPONENT_ARM_DISARM:
			arming_res = TRANSITION_NOT_CHANGED;

			if (!armed->armed && ((int)(cmd->param1 + 0.5f)) == 1) {
				if (safety->safety_switch_available && !safety->safety_off) {
					fprintf (stderr, "WARNING: Not arming, press safety switch first.\n");
					arming_res = TRANSITION_DENIED;

				} else {
					arming_res = arming_state_transition(status, safety, control_flags, arming_state_armed, armed);
				}

				if (arming_res == TRANSITION_CHANGED) {
					//mavlink_log_critical(mavlink_fd, "#audio: ARMED by component arm cmd");
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					//mavlink_log_critical(mavlink_fd, "#audio: REJECTING component arm cmd");
					result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}

			break;

		case VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
			if (is_safe(status, safety, armed)) {

				if (((int)(cmd->param1)) == 1) {
					// answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					usleep(100000);

					/* reboot */
					// xxx NOT YET IMPLEMENTED

				} else if (((int)(cmd->param1)) == 3) {
					// answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
					usleep(100000);
					/* reboot to bootloader */
					// xxx NOT YET IMPLEMENTED

				} else {
					// answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
				}

			} else {
				// answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
			}

			break;

		case VEHICLE_CMD_PREFLIGHT_CALIBRATION:
			/* try to go to INIT/PREFLIGHT arming state */

			// XXX disable interrupts in arming_state_transition
			if (TRANSITION_DENIED == arming_state_transition(status, safety, control_flags, arming_state_init, armed)) {
				// answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
				break;
			}

			if (sensors_calibration (cmd) == 0 /* OK */) {
				// tune_positive();
			} else {
				// tune_negative();
			}

			arming_state_transition(status, safety, control_flags, arming_state_standby, armed);

			break;

		default:
			break;
	}

	/* supported command handling stop */
	// if (result == VEHICLE_CMD_RESULT_ACCEPTED) {
		// tune_positive();

	// } else if (result == VEHICLE_CMD_RESULT_UNSUPPORTED) {
		// /* we do not care in the high prio loop about commands we don't know */
	// } else {
		// tune_negative();

		// if (result == VEHICLE_CMD_RESULT_DENIED) {
			// mavlink_log_critical(mavlink_fd, "#audio: command denied: %u", cmd->command);

		// } else if (result == VEHICLE_CMD_RESULT_FAILED) {
			// mavlink_log_critical(mavlink_fd, "#audio: command failed: %u", cmd->command);

		// } else if (result == VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) {
			// mavlink_log_critical(mavlink_fd, "#audio: command temporarily rejected: %u", cmd->command);

		// }
	// }

	return result;
}


int battery_remaining_estimate_voltage(float voltage, float *remaining)
{
	static float chemistry_voltage_empty;
	static float chemistry_voltage_full;
	float ncells;
	float ret = 0;

	if (!battery_initialized) {
		bat_volt_empty = param_find("BAT_V_EMPTY");
		bat_volt_full = param_find("BAT_V_FULL");
		bat_n_cells = param_find("BAT_N_CELLS");

		if (bat_volt_empty == PARAM_INVALID ||
			bat_volt_full == PARAM_INVALID ||
			bat_n_cells == PARAM_INVALID)
		{
			return -1;
		}

		battery_initialized = 1 /* true */;
	}

	if (battery_initialized && battery_counter % 100 == 0) {
		PARAM_GET(bat_volt_empty, &chemistry_voltage_empty);
		PARAM_GET(bat_volt_full, &chemistry_voltage_full);
		PARAM_GET(bat_n_cells, &ncells);
	}

	battery_counter++;

	ret = (voltage - ncells * chemistry_voltage_empty) / (ncells * (chemistry_voltage_full - chemistry_voltage_empty));

	/* limit to sane values */
	ret = (ret < 0.0f) ? 0.0f : (ret > 1.0f) ? 1.0f : ret;

#if 0
	if (ret < 0.5)
	{
		fprintf (stderr, "battery_remaining_estimate_voltage - ret\t%f\n", ret);
		fprintf (stderr, "battery_remaining_estimate_voltage - voltage\t%f\n", voltage);
		fprintf (stderr, "battery_remaining_estimate_voltage - ncells\t%f\n", ncells);
		fprintf (stderr, "battery_remaining_estimate_voltage - chemistry_voltage_empty\t%f\n", chemistry_voltage_empty);
		fprintf (stderr, "battery_remaining_estimate_voltage - chemistry_voltage_empty\t%f\n", chemistry_voltage_full);
		fprintf (stderr, "battery_remaining_estimate_voltage - numerator\t%f\n", (voltage - ncells * chemistry_voltage_empty));
		fprintf (stderr, "battery_remaining_estimate_voltage - denominator\t%f\n", (ncells * (chemistry_voltage_full - chemistry_voltage_empty)));
	}
#endif

	*remaining = ret;

	return 0;
}


/**
 * Mainloop of commander.
 */
void* commander_thread_main (void* args)
{
	/* welcome user */
	fprintf (stdout, "Commander started\n");
	fflush(stdout);

	absolute_time t;
	absolute_time last_modification_timestamp;
	transition_result_t res;	// store all transitions results here

	unsigned counter = 0;
	unsigned low_voltage_counter = 0;
	unsigned critical_voltage_counter = 0;
	absolute_time battery_info_time = 0;
	absolute_time want_to_arm_time = 0;

	uint64_t start_time = 0;
	bool_t low_battery_voltage_actions_done = 0 /* false */;
	bool_t critical_battery_voltage_actions_done = 0 /* false */;
	bool_t status_changed = 1 /* true */;
	//bool_t param_init_forced = 1 /* true */;
	bool_t updated = 0 /* false */;
	bool_t hil_enabled = 0 /* false */;

	/* not yet initialized */
	commander_initialized = 0 /* false */;
	bool_t home_position_set = 0 /* false */;
	bool_t takeoff_position_set = 0 /* false */;


	/* topics */
	/* armed topic */
	struct actuator_armed_s armed;	//armed topic
	memset ((void *) &armed, 0, sizeof(armed));

	/* control flags topic */
	/* Initialize all control flags to false */
	struct vehicle_control_flags_s control_flags;	//flags for control apps
	memset ((void *) &control_flags, 0, sizeof(control_flags));
	// just disable offboard control for now
	control_flags.flag_control_offboard_enabled = 0 /* false */;
	control_flags.flag_external_manual_override_ok = 1 /* true */;

	/* Main state machine */
	/* make sure we are in preflight state */
	struct vehicle_status_s status;	//vehicle status topic
	memset ((void *) &status, 0, sizeof(status));
	status.condition_landed = 1 /* true */;	// initialize to safe value
	status.main_flight_mode = main_state_manual;
	status.navigation_state = navigation_state_direct;
	status.finite_state_machine = finite_state_machine_preflight;
	status.arming_state = arming_state_init;
	status.hil_state = hil_state_off;
	status.is_rotary_wing = 0 /* false */;	// only fixedwing

	/* neither manual nor offboard control commands have been received */
	status.rc_signal_found_once = 0 /* false */;
	status.offboard_control_signal_found_once = 0 /* false */;

	/* mark all signals lost as long as they haven't been found */
	status.rc_signal_lost = 1 /* true */;
	status.offboard_control_signal_lost = 1 /* true */;

	/* set battery warning flag */
	status.battery_warning = battery_warning_none;
	status.condition_battery_voltage_valid = 0 /* false */;

	// for now just set sensors as initialized
	status.condition_system_sensors_initialized = 1 /* true */;


	/* advertise to ORB and publish current state machine */
	status_adv = orb_advertise(ORB_ID(vehicle_status));
	if (status_adv < 0) {
		fprintf (stderr, "Commander thread failed to advertise vehicle_status topic\n");
		exit(-1);
	}
	orb_publish(ORB_ID(vehicle_status), status_adv, &status);

	armed_adv = orb_advertise(ORB_ID(actuator_armed));
	if (armed_adv < 0) {
		fprintf (stderr, "Commander thread failed to advertise actuator_armed topic\n");
		exit(-1);
	}
	orb_publish(ORB_ID(actuator_armed), armed_adv, &armed);

	control_flags_adv = orb_advertise(ORB_ID(vehicle_control_flags));
	if (control_flags_adv < 0) {
		fprintf (stderr, "Commander thread failed to advertise vehicle_control_flags topic\n");
		exit(-1);
	}
	orb_publish(ORB_ID(vehicle_control_flags), control_flags_adv, &control_flags);

	/* home position */
	orb_advert_t home_position_adv = orb_advertise(ORB_ID(home_position));
	if (home_position_adv < 0) {
		fprintf (stderr, "Commander thread failed to advertise home_position topic\n");
		exit(-1);
	}
	struct home_position_s home_position;
	memset ((void *) &home_position, 0, sizeof(home_position));

	/* home position */
	orb_advert_t takeoff_position_adv = orb_advertise(ORB_ID(takeoff_position));
	if (takeoff_position_adv < 0) {
		fprintf (stderr, "Commander thread failed to advertise takeoff_position topic\n");
		exit(-1);
	}
	struct takeoff_position_s takeoff_position;
	memset ((void *) &takeoff_position, 0, sizeof(takeoff_position));


	//mavlink_log_info(mavlink_fd, "[cmd] started");

	/* Subscribe to safety topic */
	int safety_sub = orb_subscribe(ORB_ID(safety));
	struct safety_s safety;
	memset ((void *) &safety, 0, sizeof(safety));
	safety.safety_switch_available = 0 /* false */;
	safety.safety_off = 0 /* false */;
	if (safety_sub < 0)
	{
		fprintf (stderr, "Commander thread failed to subscribe to safety topic\n");
		exit(-1);
	}

	/* Subscribe to manual control data */
	int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s sp_man;
	memset ((void *) &sp_man, 0, sizeof(sp_man));
	if (sp_man_sub < 0)
	{
		fprintf (stderr, "Commander thread failed to subscribe to manual_control_setpoint topic\n");
		exit(-1);
	}

	/* Subscribe to global position */
	int global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_s global_position;
	memset ((void *) &global_position, 0, sizeof(global_position));
	if (global_position_sub < 0)
	{
		fprintf (stderr, "Commander thread failed to subscribe to vehicle_global_position topic\n");
		exit(-1);
	}

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset ((void *) &cmd, 0, sizeof(cmd));
	if (cmd_sub < 0)
	{
		fprintf (stderr, "Commander thread failed to subscribe to vehicle_command topic\n");
		exit(-1);
	}

	/* Subscribe to parameters changed topic */
	int param_changed_sub = orb_subscribe(ORB_ID(parameter_update));
	struct parameter_update_s param_changed;
	memset ((void *) &param_changed, 0, sizeof(param_changed));
	if (param_changed_sub < 0)
	{
		fprintf (stderr, "Commander thread failed to subscribe to parameter_update topic\n");
		exit(-1);
	}

	/* Subscribe to battery topic */
	int battery_sub = orb_subscribe(ORB_ID(battery_status));
	int battery_remaining_estimation_return_value;
	float battery_remaining_estimation;
	struct battery_status_s battery;
	memset ((void *) &battery, 0, sizeof(battery));
	battery.voltage_v = 0.0f;
	if (battery_sub < 0)
	{
		fprintf (stderr, "Commander thread failed to subscribe to battery_status topic\n");
		exit(-1);
	}

	/* Subscribe to subsystem info topic */
	int subsys_sub = orb_subscribe(ORB_ID(subsystem_info));
	struct subsystem_info_s info;
	memset ((void *) &info, 0, sizeof(info));
	if (subsys_sub < 0)
	{
		fprintf (stderr, "Commander thread failed to subscribe to subsystem_info topic\n");
		exit(-1);
	}


	/* set parameters */
	param_t _param_takeoff_alt = PARAM_INVALID;
	param_t _param_hil_enabled = PARAM_INVALID;
	bool_t parameters_initialized = 0;


	/* now initialized */
	commander_initialized = 1 /* true */;
	start_time = get_absolute_time();


	while (!_shutdown_all_systems) {
		t = get_absolute_time();

		/* update parameters */
		updated = orb_check(ORB_ID(parameter_update), param_changed_sub);
		if (updated) {
			if (!parameters_initialized)
			{
				_param_takeoff_alt = param_find("TAKEOFF_ALT");
				_param_hil_enabled = param_find("HIL_ENABLED");

				if (_param_takeoff_alt != PARAM_INVALID && _param_hil_enabled != PARAM_INVALID)
					parameters_initialized = 1;
			}

			if (parameters_initialized)
			{
				orb_copy(ORB_ID(parameter_update), param_changed_sub, &param_changed);

				param_get(_param_takeoff_alt, &takeoff_alt);
				param_get(_param_hil_enabled, &hil_enabled);

				if (hil_enabled && status.hil_state == hil_state_off)
				{
					res = hil_state_transition(hil_state_on, status_adv, &status, control_flags_adv, &control_flags, armed_adv, &armed/*, mavlink_fd*/);
					if (res == TRANSITION_DENIED)
						fprintf (stderr, "Not switching off HIL (safety)\n");
				}
			}
		}


		/* update manual_control_setpoint topic */
		updated = orb_check(ORB_ID(manual_control_setpoint), sp_man_sub);
		if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
		}

		/* update safety topic */
		updated = orb_check(ORB_ID(safety), safety_sub);
		if (updated) {
			orb_copy(ORB_ID(safety), safety_sub, &safety);

			/* disarm if safety is now on and still armed */
			if (safety.safety_switch_available && !safety.safety_off) {
			 	if (status.arming_state == arming_state_armed) {
			 		fprintf (stderr, "INFO: Disarmed by the user\n");
			 	}

			 	(void)arming_state_transition(&status, &safety, &control_flags, arming_state_standby, &armed);
			}
		}

		/* update subsystem */
		updated = orb_check(ORB_ID(subsystem_info), subsys_sub);
		if (updated) {
			orb_copy(ORB_ID(subsystem_info), subsys_sub, &info);

			fprintf (stderr, "subsystem changed: %d\n", (int)info.subsystem_type);

			// /* mark / unmark as present */
			if (info.present) {
				status.onboard_control_sensors_present |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_present &= ~info.subsystem_type;
			}

			// /* mark / unmark as enabled */
			if (info.enabled) {
				status.onboard_control_sensors_enabled |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_enabled &= ~info.subsystem_type;
			}

			// /* mark / unmark as ok */
			if (info.ok) {
				status.onboard_control_sensors_health |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_health &= ~info.subsystem_type;
			}

			status_changed = 1 /* true */;
		}


		/* update battery status */
		updated = orb_check(ORB_ID(battery_status), battery_sub);
		if (updated) {
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);
			// fprintf (stderr, "bat v: %2.2f", battery.voltage_v);

			/* only consider battery voltage if system has been running 2s and battery voltage is higher than 4V */
			if (get_absolute_time() > start_time + 2000000 && battery.voltage_v > 4.0f) {
				battery_remaining_estimation_return_value = battery_remaining_estimate_voltage(battery.voltage_v, &battery_remaining_estimation);

				status.condition_battery_voltage_valid = (battery_remaining_estimation_return_value == 0)? 1 /* true */ : 0 /* false */;
				status.battery_remaining = (status.condition_battery_voltage_valid)? battery_remaining_estimation : 1;
				//fprintf (stderr, "bat remaining: %2.2f", status.battery_remaining);

				if (status.condition_battery_voltage_valid && t > battery_info_time + BATTERY_INFO_EXPORT_TIME) {
					fprintf (stdout, "INFO: Battery remaining  %.1f%% \n", battery_remaining_estimation*100);
					battery_info_time = t;
				}
			}
		}

		/* if battery voltage is getting lower, warn using buzzer, etc. */
		if (status.condition_battery_voltage_valid && status.battery_remaining < 0.25f && !low_battery_voltage_actions_done) {
			//TODO: add filter, or call emergency after n measurements < VOLTAGE_BATTERY_MINIMAL_MILLIVOLTS
			if (low_voltage_counter > LOW_VOLTAGE_BATTERY_COUNTER_LIMIT) {
				low_battery_voltage_actions_done = 1 /* true */;
				//mavlink_log_critical(mavlink_fd, "#audio: WARNING: LOW BATTERY");
				// battery_tune_played = 0 /* false */;
				status.battery_warning = battery_warning_low;
				status_changed = 1 /* true */;
				fprintf (stderr, "EMERGENCY: LOW BATTERY LEVEL\n");
			}

			low_voltage_counter++;

		} else if (status.condition_battery_voltage_valid && status.battery_remaining < 0.1f && !critical_battery_voltage_actions_done && low_battery_voltage_actions_done) {
			/* critical battery voltage, this is rather an emergency, change state machine */
			if (critical_voltage_counter > CRITICAL_VOLTAGE_BATTERY_COUNTER_LIMIT) {
				critical_battery_voltage_actions_done = 1 /* true */;
				//mavlink_log_critical(mavlink_fd, "#audio: EMERGENCY: CRITICAL BATTERY");
				// battery_tune_played = 0 /* false */;
				status.battery_warning = battery_warning_critical;
				fprintf (stderr, "EMERGENCY: CRITICAL BATTERY LEVEL\n");

				if (armed.armed) {
					arming_state_transition(&status, &safety, &control_flags, arming_state_armed_error, &armed);

				} else {
					arming_state_transition(&status, &safety, &control_flags, arming_state_standby_error, &armed);
				}

				status_changed = 1 /* true */;
			}

			critical_voltage_counter++;

		} else {

			low_voltage_counter = 0;
			critical_voltage_counter = 0;
		}
		/* End battery voltage check */


		/* update global position estimate */
		updated = orb_check(ORB_ID(vehicle_global_position), global_position_sub);
		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
		}

		/* update condition_global_position_valid */
		orb_stat(ORB_ID(vehicle_global_position), global_position_sub, &last_modification_timestamp);
		check_valid(last_modification_timestamp, POSITION_TIMEOUT, global_position.valid, &(status.condition_global_position_valid), &status_changed);

		if (status.condition_global_position_valid) {
			if (status.condition_landed != global_position.landed) {
				status.condition_landed = global_position.landed;
				status_changed = 1 /* true */;

				//if (status.condition_landed) {
					//mavlink_log_critical(mavlink_fd, "#audio: LANDED");
				//} else {
					//mavlink_log_critical(mavlink_fd, "#audio: IN AIR");
				//}
			}
		}

		/*
		 * Check for valid position information.
		 *
		 * If home position is not yet set AND the last
		 * GPS measurement is not older than one seconds AND
		 * the system is currently not armed, set home
		 * position to the current position.
		 */
		if (updated && status.condition_global_position_valid && !home_position_set && global_position.landed && !armed.armed) {
			/* copy position data to ORB home message, store it locally as well */
			home_position.latitude = global_position.latitude;
			home_position.longitude = global_position.longitude;
			home_position.altitude = global_position.altitude;

			home_position.time_gps_usec = global_position.time_gps_usec;

			//fprintf (stderr, "home_position: lat = %.7f, lon = %.7f\n", home_position.latitude, home_position.longitude);
			//mavlink_log_info(mavlink_fd, "[cmd] home: %.7f, %.7f", home_lat_d, home_lon_d);

			/* announce new home position */
			orb_publish(ORB_ID(home_position), home_position_adv, &home_position);

			/* mark home position as set */
			home_position_set = 1 /* true */;
			//tune_positive();
		}

		/*
		 * Check for valid position information.
		 *
		 * If home position is not yet set AND the last
		 * GPS measurement is not older than one seconds AND
		 * the system is currently not armed, set home
		 * position to the current position.
		 */
		if (updated && status.condition_global_position_valid && !takeoff_position_set &&
			!global_position.landed && (global_position.altitude - global_position.ground_level) > takeoff_alt && armed.armed) {
			/* copy position data to ORB home message, store it locally as well */
			takeoff_position.latitude = global_position.latitude;
			takeoff_position.longitude = global_position.longitude;
			takeoff_position.altitude = global_position.altitude;

			takeoff_position.time_gps_usec = global_position.time_gps_usec;

			//fprintf (stderr, "takeoff_position: lat = %.7f, lon = %.7f\n", takeoff_position.latitude, takeoff_position.longitude);

			/* announce new takeoff position */
			orb_publish(ORB_ID(takeoff_position), takeoff_position_adv, &takeoff_position);

			/* mark takeoff position as set */
			takeoff_position_set = 1 /* true */;
			//tune_positive();
		}


		/* If in INIT state, try to proceed to STANDBY state */
		if (status.arming_state == arming_state_init) {
			if (sensors_calibration ((vehicle_command_s *) NULL) == 0) {
				arming_state_transition(&status, &safety, &control_flags, arming_state_standby, &armed);
			}
		}


		/* ignore RC signals if in offboard control mode */
		orb_stat(ORB_ID(manual_control_setpoint), sp_man_sub, &last_modification_timestamp);
		if (!status.offboard_control_signal_found_once && last_modification_timestamp != 0) {

#ifndef ALWAYS_CONSIDER_MAN_SP_AS_VALID
			/* start RC input check */
			if (get_absolute_time() < last_modification_timestamp + RC_TIMEOUT) {
#endif
				/* handle the case where RC signal was regained */
				if (!status.rc_signal_found_once) {
					status.rc_signal_found_once = 1;
					//mavlink_log_critical(mavlink_fd, "#audio: detected RC signal first time");
					fprintf (stdout, "Detected RC signal for the first time\n");
					fflush (stdout);
					status_changed = 1;

				} else {
					if (status.rc_signal_lost) {
						//mavlink_log_critical(mavlink_fd, "#audio: RC signal regained");
						fprintf (stderr, "RC signal regained\n");
						status_changed = 1;
					}
				}

				status.rc_signal_lost = 0;

				/* arm/disarm by RC */
				res = TRANSITION_NOT_CHANGED;

				if (sp_man.want_to_arm && sp_man.throttle < STICK_THRUST_RANGE * 0.1f && status.arming_state == arming_state_standby) {
					if (want_to_arm_time == 0) {
						want_to_arm_time = t;
					} else if (t > want_to_arm_time + WANT_TO_ARM_TIME_COUNTER_LIMIT) {
						if (safety.safety_switch_available && !safety.safety_off) {
							if (counter % (200000 / COMMANDER_ERROR_OUTPUT_INTERVAL) == 0)
								fprintf (stderr, "WARNING: Not arming, press safety switch first.\n");
						} else if (status.main_flight_mode != main_state_manual) {
							if (counter % (200000 / COMMANDER_ERROR_OUTPUT_INTERVAL) == 0)
								fprintf (stderr, "WARNING: Not arming, switch to manual mode first.\n");
						} else {
							res = arming_state_transition(&status, &safety, &control_flags, arming_state_armed, &armed);

							want_to_arm_time = 0;
						}
					}
				} else {
					want_to_arm_time = 0;
				}

				if (res == TRANSITION_CHANGED) {
					if (status.arming_state == arming_state_armed) {
						fprintf (stderr, "INFO: Armed by the user\n");

					} else {
						fprintf (stderr, "INFO: Disarmed by the user\n");
					}
				} else if (res == TRANSITION_DENIED) {
					fprintf (stderr, "ERROR: arming transition denied: arm %d main %d sub %d\n", status.arming_state, status.main_flight_mode, status.sub_flight_mode);
					//mavlink_log_critical(mavlink_fd, "#audio: ERROR: main denied: arm %d main %d mode_sw %d", status.arming_state, status.main_flight_mode, status.mode_switch);
				}

				/* evaluate the main state machine */
				res = check_main_state_machine(&sp_man, &status);
				if (res == TRANSITION_CHANGED) {
					//mavlink_log_info(mavlink_fd, "[cmd] main state: %d", status.main_flight_mode);
					//tune_positive();

				} else if (res == TRANSITION_DENIED) {
					/* DENIED here indicates bug in the commander */
					fprintf (stderr, "ERROR: main state transition denied: arm %d main %d sub %d\n", status.arming_state, status.main_flight_mode, status.sub_flight_mode);
					//mavlink_log_critical(mavlink_fd, "#audio: ERROR: main denied: arm %d main %d mode_sw %d", status.arming_state, status.main_flight_mode, status.mode_switch);
				}

#ifndef ALWAYS_CONSIDER_MAN_SP_AS_VALID
			} else {
				if (!status.rc_signal_lost) {
					//mavlink_log_critical(mavlink_fd, "#audio: CRITICAL: RC SIGNAL LOST");
					fprintf (stderr, "RC signal lost\n");
					status.rc_signal_lost = 1;
					status_changed = 1;
				}
			}
#endif
		}


		/* handle commands last, as the system needs to be updated to handle them */
		updated = orb_check(ORB_ID(vehicle_command), cmd_sub);
		if (updated) {
			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle it */
			handle_command(&status, &safety, &control_flags, &cmd, &armed);
		}


		/* evaluate the navigation state machine */
		res = check_navigation_state_machine(&status, &control_flags, &global_position);
		if (res == TRANSITION_DENIED) {
			/* DENIED here indicates bug in the commander */
			fprintf (stderr, "ERROR: nav denied: arm %d main %d nav %d\n", status.arming_state, status.main_flight_mode, status.navigation_state);
			//mavlink_log_critical(mavlink_fd, "#audio: ERROR: nav denied: arm %d main %d nav %d", status.arming_state, status.main_flight_mode, status.navigation_state);
		}

		/* check which state machines for changes, clear "changed" flag */
		bool_t arming_state_changed = check_arming_state_changed();
		bool_t main_state_changed = check_main_state_changed();
		bool_t navigation_state_changed = check_navigation_state_changed();

		if (navigation_state_changed || arming_state_changed) {
			control_flags.flag_armed = armed.armed;	// copy armed state to vehicle_control_flags topic
		}

		if (arming_state_changed || main_state_changed || navigation_state_changed) {
			//mavlink_log_info(mavlink_fd, "[cmd] state: arm %d, main %d, nav %d", status.arming_state, status.main_flight_mode, status.navigation_state);
			status_changed = 1 /* true */;
		}

		/* publish states (armed, control mode, vehicle status) at least with 10 Hz */
		counter++;
		if (status_changed || (counter) % (200000 / COMMANDER_MONITORING_INTERVAL) == 0) {
			orb_publish(ORB_ID(vehicle_status), status_adv, &status);
			orb_publish(ORB_ID(vehicle_control_flags), control_flags_adv, &control_flags);
			orb_publish(ORB_ID(actuator_armed), armed_adv, &armed);
		}

		status_changed = 0 /* false */;
		usleep(COMMANDER_MONITORING_INTERVAL);
	}


	/* unsubscribe topics */
	orb_unsubscribe(ORB_ID(manual_control_setpoint), sp_man_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_global_position), global_position_sub, pthread_self());
	orb_unsubscribe(ORB_ID(safety), safety_sub, pthread_self());
	orb_unsubscribe(ORB_ID(vehicle_command), cmd_sub, pthread_self());
	orb_unsubscribe(ORB_ID(parameter_update), param_changed_sub, pthread_self());
	orb_unsubscribe(ORB_ID(subsystem_info), subsys_sub, pthread_self());
	orb_unsubscribe(ORB_ID(battery_status), battery_sub, pthread_self());

	/* unadvertise topics */
	orb_unadvertise(ORB_ID(vehicle_control_flags), control_flags_adv, pthread_self());
	orb_unadvertise(ORB_ID(vehicle_status), status_adv, pthread_self());
	orb_unadvertise(ORB_ID(actuator_armed), armed_adv, pthread_self());
	orb_unadvertise(ORB_ID(home_position), home_position_adv, pthread_self());
	orb_unadvertise(ORB_ID(takeoff_position), takeoff_position_adv, pthread_self());

	return 0;
}
