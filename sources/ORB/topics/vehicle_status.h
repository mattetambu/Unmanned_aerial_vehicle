/**
 * @file vehicle_status.h
 * Definition of the vehicle_status ORB topic.
 *
 * Published the state machine and the system status bitfields
 * (see SYS_STATUS mavlink message), published only by commander app.
 *
 * All apps should write to subsystem_info:
 *
 *  (any app) --> subsystem_info (published) --> (commander app state machine)  --> vehicle_status --> (mavlink app)
 */

#ifndef VEHICLE_STATUS_H_
#define VEHICLE_STATUS_H_

	#include <stdint.h>
	#include "../ORB.h"
	#include "../../uav_library/common.h"

	
	/**
	 * @addtogroup topics @{
	 */

	/* main flight mode */
	typedef enum main_flight_mode_t
	{
		main_state_manual = 0,
		main_state_support,
		main_state_auto,
	} main_flight_mode_t;

	/* exact flight mode */
	typedef enum sub_flight_mode_t
	{
		sub_flight_mode_manual,
		sub_flight_mode_stabilize,
		sub_flight_mode_fly_by_wire,
		sub_flight_mode_easy,
		sub_flight_mode_loiter,
		sub_flight_mode_fly_to,
		sub_flight_mode_rtl,
		sub_flight_mode_mission
	} sub_flight_mode_t;

	inline static const char* sub_flight_mode_to_string (sub_flight_mode_t index)
	{
		switch(index)
		{
			return_custom_enum_string (sub_flight_mode_manual, "manual");
			return_custom_enum_string (sub_flight_mode_stabilize, "stabilize");
			return_custom_enum_string (sub_flight_mode_fly_by_wire, "fly_by_wire");
			return_custom_enum_string (sub_flight_mode_easy, "easy");
			return_custom_enum_string (sub_flight_mode_loiter, "loiter");
			return_custom_enum_string (sub_flight_mode_fly_to, "fly_to");
			return_custom_enum_string (sub_flight_mode_rtl, "rtl");
			return_custom_enum_string (sub_flight_mode_mission, "mission");
			default: return NULL;
		}
	}

	/* navigation state */
	typedef enum navigation_state_t
	 {
		navigation_state_direct = 0,	// true manual control, no any stabilization
		navigation_state_stabilize,		// attitude stabilization
		navigation_state_althold,		// attitude + altitude stabilization
		navigation_state_vector,		// attitude + altitude + position stabilization
		navigation_state_auto_ready,	// auto, landed, ready for takeoff
		navigation_state_auto_takeoff,	// detect takeoff using land detector and switch to desired auto mode
		navigation_state_auto_loiter,	// pause mission
		navigation_state_auto_mission,	// fly mission
		navigation_state_auto_rtl,		// return to launch, when home position switch to land
		navigation_state_auto_land		// land and switch to auto_ready when landed (detect using land detector)
	} navigation_state_t;

	inline static const char* navigation_state_to_string (navigation_state_t index)
	{
		switch(index)
		{
			return_custom_enum_string (navigation_state_direct, "direct");
			return_custom_enum_string (navigation_state_stabilize, "stabilize");
			return_custom_enum_string (navigation_state_althold, "althold");
			return_custom_enum_string (navigation_state_vector, "vector");
			return_custom_enum_string (navigation_state_auto_ready, "auto_ready");
			return_custom_enum_string (navigation_state_auto_takeoff, "auto_takeoff");
			return_custom_enum_string (navigation_state_auto_loiter, "auto_loiter");
			return_custom_enum_string (navigation_state_auto_mission, "auto_mission");
			return_custom_enum_string (navigation_state_auto_rtl, "auto_rtl");
			return_custom_enum_string (navigation_state_auto_land, "auto_land");
			default: return NULL;
		}
	}

	/* arming machine */
	typedef enum arming_state_t
	{
		arming_state_init = 0,
		arming_state_standby,
		arming_state_standby_error,
		arming_state_armed,
		arming_state_armed_error,
		arming_state_in_air_restore,
		arming_state_reboot
	} arming_state_t;

	inline static const char* arming_state_to_string (arming_state_t index)
	{
		switch(index)
		{
			return_custom_enum_string (arming_state_init, "init");
			return_custom_enum_string (arming_state_standby, "standby");
			return_custom_enum_string (arming_state_standby_error, "standby_error");
			return_custom_enum_string (arming_state_armed, "armed");
			return_custom_enum_string (arming_state_armed_error, "armed_error");
			return_custom_enum_string (arming_state_in_air_restore, "in_air_restore");
			return_custom_enum_string (arming_state_reboot, "reboot");
			default: return NULL;
		}
	}

	/* finite state machine */
	typedef enum finite_state_machine_t
	{
		finite_state_machine_preflight = 0,
		finite_state_machine_arming,
		finite_state_machine_in_flight,
		finite_state_machine_warning_recover,
		finite_state_machine_mission_abort,
		finite_state_machine_emergency_landing,
		finite_state_machine_emergency_cutoff
	} finite_state_machine_t;

	inline static const char* finite_state_machine_to_string (finite_state_machine_t index)
	{
		switch(index)
		{
			return_custom_enum_string (finite_state_machine_preflight, "preflight");
			return_custom_enum_string (finite_state_machine_arming, "arming");
			return_custom_enum_string (finite_state_machine_in_flight, "in_flight");
			return_custom_enum_string (finite_state_machine_warning_recover, "warning_recover");
			return_custom_enum_string (finite_state_machine_mission_abort, "mission_abort");
			return_custom_enum_string (finite_state_machine_emergency_landing, "emergency_landing");
			return_custom_enum_string (finite_state_machine_emergency_cutoff, "emergency_cutoff");
			default: return NULL;
		}
	}

	typedef enum hil_state_t
	{
		hil_state_off = 0,
		hil_state_on
	} hil_state_t;
	
	typedef enum battery_warning_t
	{
		battery_warning_none = 0,	/**< no battery low voltage warning active */
		battery_warning_low,		/**< warning of low voltage */
		battery_warning_critical	/**< alerting of critical voltage */
	} battery_warning_t;

	
	enum VEHICLE_MODE_FLAG {
		VEHICLE_MODE_FLAG_SAFETY_ARMED = 128,
		VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
		VEHICLE_MODE_FLAG_HIL_ENABLED = 32,
		VEHICLE_MODE_FLAG_STABILIZED_ENABLED = 16,
		VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8,
		VEHICLE_MODE_FLAG_AUTO_ENABLED = 4,
		VEHICLE_MODE_FLAG_TEST_ENABLED = 2,
		VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
	}; /**< Same as MAV_MODE_FLAG of MAVLink 1.0 protocol */

	/**
	 * Should match 1:1 MAVLink's MAV_TYPE ENUM
	 */
	enum VEHICLE_TYPE {
		VEHICLE_TYPE_GENERIC=0, /* Generic micro air vehicle. | */
		VEHICLE_TYPE_FIXED_WING=1, /* Fixed wing aircraft. | */
		VEHICLE_TYPE_QUADROTOR=2, /* Quadrotor | */
		VEHICLE_TYPE_COAXIAL=3, /* Coaxial helicopter | */
		VEHICLE_TYPE_HELICOPTER=4, /* Normal helicopter with tail rotor. | */
		VEHICLE_TYPE_ANTENNA_TRACKER=5, /* Ground installation | */
		VEHICLE_TYPE_GCS=6, /* Operator control unit / ground control station | */
		VEHICLE_TYPE_AIRSHIP=7, /* Airship, controlled | */
		VEHICLE_TYPE_FREE_BALLOON=8, /* Free balloon, uncontrolled | */
		VEHICLE_TYPE_ROCKET=9, /* Rocket | */
		VEHICLE_TYPE_GROUND_ROVER=10, /* Ground rover | */
		VEHICLE_TYPE_SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
		VEHICLE_TYPE_SUBMARINE=12, /* Submarine | */
		VEHICLE_TYPE_HEXAROTOR=13, /* Hexarotor | */
		VEHICLE_TYPE_OCTOROTOR=14, /* Octorotor | */
		VEHICLE_TYPE_TRICOPTER=15, /* Octorotor | */
		VEHICLE_TYPE_FLAPPING_WING=16, /* Flapping wing | */
		VEHICLE_TYPE_KITE=17, /* Kite | */
		VEHICLE_TYPE_ENUM_END=18, /*  | */
	};

	/**
	 * @addtogroup topics
	 * @{
	 */

	/**
	 * Status of the vehicle.
	 *
	 * Encodes the complete system state and is set by the commander app.
	 */
	struct vehicle_status_s
	{
		main_flight_mode_t main_flight_mode;			/**< main flight mode */
		sub_flight_mode_t sub_flight_mode;			/**< exact flight mode */
		navigation_state_t navigation_state;			/**< navigation state */
		arming_state_t arming_state;					/**< arming state */
		finite_state_machine_t finite_state_machine;	/**< finite state machine */
		hil_state_t hil_state;							/**< current hil state */

		battery_warning_t battery_warning;				/**< current battery warning mode, as defined by VEHICLE_BATTERY_WARNING enum */
		int battery_remaining;
		
		//int32_t system_type;			/**< system type, inspired by MAVLink's VEHICLE_TYPE enum */
		//int32_t system_id;				/**< system id, inspired by MAVLink's system ID field */
		//int32_t component_id;			/**< subsystem / component id, inspired by MAVLink's component ID field */
		bool_t is_rotary_wing;

		bool_t condition_auto_mission_available;
		bool_t condition_global_position_valid;		/**< set to true by the commander app if the quality of the gps signal is good enough to use it in the position estimator */
		bool_t condition_local_position_valid;
		bool_t condition_launch_position_valid;		/**< indicates a valid launch position */
		bool_t condition_home_position_valid;			/**< indicates a valid home position (a valid home position is not always a valid launch) */
		bool_t condition_local_altitude_valid;
		bool_t condition_airspeed_valid;				/**< set to true by the commander app if there is a valid airspeed measurement available */
		
		bool_t condition_system_returned_to_home;
		bool_t condition_landed;						/**< true if vehicle is landed, always true if disarmed */
		bool_t condition_battery_voltage_valid;

		bool_t condition_system_sensors_initialized;
		bool_t condition_system_in_air_restore;		/**< true if we can restore in mid air */

		/* see SYS_STATUS mavlink message for the following */
		uint32_t onboard_control_sensors_present;
		uint32_t onboard_control_sensors_enabled;
		uint32_t onboard_control_sensors_health;
	};

	/**
	 * @}
	 */

	/* register this as object request broker structure */
	ORB_DECLARE(vehicle_status);

#endif
