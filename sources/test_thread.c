// test_thread.c

#include <sys/types.h>
#include <pthread.h>
#include <semaphore.h>
#include "test_thread.h"
#include "uav_library/common.h"
#include "uav_library/io_ctrl/comunicator.h"
#include "uav_library/console_controller/console_controller.h"

#include "ORB/ORB.h"
#include "ORB/topics/safety.h"
#include "ORB/topics/setpoint/manual_control_setpoint.h"
#include "ORB/topics/position/home_position.h"
#include "ORB/topics/position/takeoff_position.h"
#include "ORB/topics/vehicle_status.h"
#include "ORB/topics/vehicle_control_flags.h"
#include "ORB/topics/vehicle_attitude.h"
#include "ORB/topics/vehicle_hil_attitude.h"
#include "ORB/topics/position/vehicle_global_position.h"
#include "ORB/topics/actuator/actuator_armed.h"
#include "ORB/topics/parameter_update.h"

#include "uav_library/math/tests/Vector_test.h"
#include "uav_library/math/tests/Vector2f_test.h"
#include "uav_library/math/tests/Vector3f_test.h"
#include "uav_library/math/tests/EulerAngles_test.h"
#include "uav_library/math/tests/Quaternion_test.h"
#include "uav_library/math/tests/Matrix_test.h"
#include "uav_library/math/tests/Dcm_test.h"


pthread_t test_thread_id, test_thread_id2;



void* test_thread_body (void* args)
{
	int updated;
	static unsigned int TEST_THREAD_MONITORING_INTERVAL = 10;	// s

	/* ********************** Subscribe to the topics ****************************** */
	/* Subscribe to safety topic */
	int safety_sub = orb_subscribe(ORB_ID(safety));
	struct safety_s safety;
	memset ((void *) &safety, 0, sizeof(safety));
	if (safety_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to safety topic\n");
		return 0;
	}

	/* Subscribe to vehicle_control_flags topic */
	int manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s manual_control;
	memset ((void *) &manual_control, 0, sizeof(manual_control));
	if (manual_control_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to manual_control_setpoint topic\n");
		return 0;
	}

	/* Subscribe to vehicle_control_flags topic */
	int flags_sub = orb_subscribe(ORB_ID(vehicle_control_flags));
	struct vehicle_control_flags_s control_flags;
	memset ((void *) &control_flags, 0, sizeof(control_flags));
	if (flags_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to vehicle_control_flags topic\n");
		return 0;
	}

	/* Subscribe to vehicle_status topic */
	int status_sub = orb_subscribe(ORB_ID(vehicle_status));
	struct vehicle_status_s vehicle_status;
	memset ((void *) &vehicle_status, 0, sizeof(vehicle_status));
	if (status_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to vehicle_status topic\n");
		return 0;
	}

	/* Subscribe to actuator_armed topic */
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	struct actuator_armed_s actuator_armed;
	memset ((void *) &actuator_armed, 0, sizeof(actuator_armed));
	if (armed_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to actuator_armed topic\n");
		return 0;
	}

	/* Subscribe to home_position topic */
	int home_position_sub = orb_subscribe(ORB_ID(home_position));
	struct home_position_s home_position;
	memset ((void *) &home_position, 0, sizeof(home_position));
	if (home_position_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to home_position topic\n");
		return 0;
	}

	/* Subscribe to takeoff_position topic */
	int takeoff_position_sub = orb_subscribe(ORB_ID(takeoff_position));
	struct takeoff_position_s takeoff_position;
	memset ((void *) &takeoff_position, 0, sizeof(takeoff_position));
	if (takeoff_position_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to takeoff_position topic\n");
		return 0;
	}

	/* Subscribe to parameters changed topic */
	int param_changed_sub = orb_subscribe(ORB_ID(parameter_update));
	struct parameter_update_s param_changed;
	memset ((void *) &param_changed, 0, sizeof(param_changed));
	if (param_changed_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to parameter_update topic\n");
		exit(-1);
	}

	fprintf (stdout, "\n\n");
	fflush (stdout);

	while (!_shutdown_all_systems)
	{
		/* do not plot until the user has finish to control the console */
		get_console_unique_control ();

		if (_shutdown_all_systems)
			break;

		/* update safety */
		updated = orb_check(ORB_ID(safety), safety_sub);
		if (updated) {
			orb_copy(ORB_ID(safety), safety_sub, &safety);
			fprintf (stdout, "safety updated (s.safety_switch_available=%d, s.safety_off=%d)\n", (int) safety.safety_switch_available, (int) safety.safety_off);
			fflush (stdout);
		}

		/* update manual_control_setpoint */
		updated = orb_check(ORB_ID(manual_control_setpoint), manual_control_sub);
		if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), manual_control_sub, &manual_control);
			fprintf (stdout, "manual_control_setpoint updated (mc.mode_switch=%d, mc.second_switch=%d, mc.want_to_arm=%d)\n",	\
					(int) manual_control.mode_switch,	\
					(int) manual_control.second_switch,	\
					(int) manual_control.want_to_arm);
			fflush (stdout);
		}

		/* update vehicle_control_flags */
		updated = orb_check(ORB_ID(vehicle_control_flags), flags_sub);
		if (updated) {
			orb_copy(ORB_ID(vehicle_control_flags), flags_sub, &control_flags);
			fprintf (stdout, "vehicle_control_flags updated (cf.armed=%d, cf.control_manual_enabled=%d, cf.control_attitude_enabled=%d, cf.control_rates_enabled=%d, cf.control_velocity_enabled=%d, cf.control_position_enabled=%d, cf.control_altitude_enabled=%d)\n",	\
					(int) control_flags.flag_armed,						\
					(int) control_flags.flag_control_manual_enabled,	\
					(int) control_flags.flag_control_attitude_enabled,	\
					(int) control_flags.flag_control_rates_enabled,		\
					(int) control_flags.flag_control_velocity_enabled,	\
					(int) control_flags.flag_control_position_enabled,	\
					(int) control_flags.flag_control_altitude_enabled);
			fflush (stdout);
		}
		
		/* update vehicle_status */
		updated = orb_check(ORB_ID(vehicle_status), status_sub);
		if (updated) {
			orb_copy(ORB_ID(vehicle_status), status_sub, &vehicle_status);
			fprintf (stdout, "vehicle_status updated (vs.main_flight_mode=%d, vs.sub_flight_mode=%d, vs.navigation_state=%d, vs.finite_state_machine=%d, vs.arming_state=%d)\n", \
					(int) vehicle_status.main_flight_mode,	\
					(int) vehicle_status.sub_flight_mode,	\
					(int) vehicle_status.navigation_state,	\
					(int) vehicle_status.finite_state_machine,		\
					(int) vehicle_status.arming_state);
			fflush (stdout);
		}
		
		/* update actuator_armed */
		updated = orb_check(ORB_ID(actuator_armed), armed_sub);
		if (updated) {
			orb_copy(ORB_ID(actuator_armed), armed_sub, &actuator_armed);
			fprintf (stdout, "actuator_armed updated (aa.armed=%d, aa.ready_to_arm=%d, aa.lockdown=%d)\n", (int) actuator_armed.armed, (int) actuator_armed.ready_to_arm, (int) actuator_armed.lockdown);
			fflush (stdout);
		}

		/* update home_position */
		updated = orb_check(ORB_ID(home_position), home_position_sub);
		if (updated) {
			orb_copy(ORB_ID(home_position), home_position_sub, &home_position);
			fprintf (stdout, "home_position updated\n");
			fflush (stdout);
		}

		/* update takeoff_position */
		updated = orb_check(ORB_ID(takeoff_position), takeoff_position_sub);
		if (updated) {
			orb_copy(ORB_ID(takeoff_position), takeoff_position_sub, &takeoff_position);
			fprintf (stdout, "takeoff_position updated\n");
			fflush (stdout);
		}
		
		/* update parameters */
		updated = orb_check(ORB_ID(parameter_update), param_changed_sub);
		if (updated) {
			orb_copy(ORB_ID(parameter_update), param_changed_sub, &param_changed);
			fprintf (stdout, "param_changed updated\n");
			fflush (stdout);
		}

		fprintf (stdout, "\n");
		fflush (stdout);

		/* allow the user to control the console */
		release_console_control ();

		sleep(TEST_THREAD_MONITORING_INTERVAL);
	}
	
	/* **************************************** unsubscribe ****************************************** */
	orb_unsubscribe (ORB_ID(safety), safety_sub, pthread_self());
	orb_unsubscribe (ORB_ID(manual_control_setpoint), manual_control_sub, pthread_self());
	orb_unsubscribe (ORB_ID(vehicle_control_flags), flags_sub, pthread_self());
	orb_unsubscribe (ORB_ID(vehicle_status), status_sub, pthread_self());
	orb_unsubscribe (ORB_ID(actuator_armed), armed_sub, pthread_self());
	orb_unsubscribe (ORB_ID(home_position), home_position_sub, pthread_self());
	orb_unsubscribe (ORB_ID(takeoff_position), takeoff_position_sub, pthread_self());
	orb_unsubscribe (ORB_ID(parameter_update), param_changed_sub, pthread_self());
	
	return 0;
}


void* test_thread_body2 (void* args)
{
	int va_updated, vha_updated;
	absolute_time va_timestamp, vha_timestamp;
	static unsigned int TEST_THREAD_MONITORING_INTERVAL = 1;	// s


	/* ********************** Subscribe to the topics ****************************** */
	/* Subscribe to vehicle_attitude topic */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	struct vehicle_attitude_s vehicle_attitude;
	memset ((void *) &vehicle_attitude, 0, sizeof(vehicle_attitude));
	if (vehicle_attitude_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to vehicle_attitude topic\n");
		return 0;
	}

	/* Subscribe to vehicle_attitude topic */
	int vehicle_hil_attitude_sub = orb_subscribe(ORB_ID(vehicle_hil_attitude));
	struct vehicle_hil_attitude_s vehicle_hil_attitude;
	memset ((void *) &vehicle_hil_attitude, 0, sizeof(vehicle_hil_attitude));
	if (vehicle_hil_attitude_sub < 0)
	{
		fprintf (stderr, "Test thread failed to subscribe to vehicle_hil_attitude topic\n");
		return 0;
	}

	fprintf (stdout, "\n\n");
	fflush (stdout);

	while (!_shutdown_all_systems)
	{
		if (_shutdown_all_systems)
			break;

		va_updated = orb_poll(ORB_ID(vehicle_attitude), vehicle_attitude_sub, 10000000);
		if (va_updated) {
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &vehicle_attitude);
			orb_stat (ORB_ID(vehicle_attitude), vehicle_attitude_sub, &va_timestamp);
		}

		vha_updated = orb_poll(ORB_ID(vehicle_hil_attitude), vehicle_hil_attitude_sub, 10000000);
		if (vha_updated) {
			orb_copy(ORB_ID(vehicle_hil_attitude), vehicle_hil_attitude_sub, &vehicle_hil_attitude);
			orb_stat (ORB_ID(vehicle_hil_attitude), vehicle_hil_attitude_sub, &vha_timestamp);
		}

		/* do not plot until the user has finish to control the console */
		get_console_unique_control ();

		fprintf (stdout, "Hil attitude publish timestamp:\t%ld\n", (long int) va_timestamp);
		fprintf (stdout, "Estimator attitude publish timestamp:\t%ld\n", (long int) vha_timestamp);

		fprintf (stdout, "Roll angle - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.roll, vehicle_attitude.roll, vehicle_hil_attitude.roll - vehicle_attitude.roll);
		fprintf (stdout, "Pitch angle - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.pitch, vehicle_attitude.pitch, vehicle_hil_attitude.pitch - vehicle_attitude.pitch);
		fprintf (stdout, "Yaw angle - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.yaw, vehicle_attitude.yaw, vehicle_hil_attitude.yaw - vehicle_attitude.yaw);
		fprintf (stdout, "Roll rate - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.roll_rate, vehicle_attitude.roll_rate, vehicle_hil_attitude.roll_rate - vehicle_attitude.roll_rate);
		fprintf (stdout, "Pitch rate - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.pitch_rate, vehicle_attitude.pitch_rate, vehicle_hil_attitude.pitch_rate - vehicle_attitude.pitch_rate);
		fprintf (stdout, "Yaw rate - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.yaw_rate, vehicle_attitude.yaw_rate, vehicle_hil_attitude.yaw_rate - vehicle_attitude.yaw_rate);

		fprintf (stdout, "q[0] - \t\t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.q[0], vehicle_attitude.q[0], vehicle_hil_attitude.q[0] - vehicle_attitude.q[0]);
		fprintf (stdout, "q[1] - \t\t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.q[1], vehicle_attitude.q[1], vehicle_hil_attitude.q[1] - vehicle_attitude.q[1]);
		fprintf (stdout, "q[2] - \t\t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.q[2], vehicle_attitude.q[2], vehicle_hil_attitude.q[2] - vehicle_attitude.q[2]);
		fprintf (stdout, "q[3] - \t\t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.q[3], vehicle_attitude.q[3], vehicle_hil_attitude.q[3] - vehicle_attitude.q[3]);

		fprintf (stdout, "R[0][0] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[0][0], vehicle_attitude.R[0][0], vehicle_hil_attitude.R[0][0] - vehicle_attitude.R[0][0]);
		fprintf (stdout, "R[0][1] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[0][1], vehicle_attitude.R[0][1], vehicle_hil_attitude.R[0][1] - vehicle_attitude.R[0][1]);
		fprintf (stdout, "R[0][2] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[0][2], vehicle_attitude.R[0][2], vehicle_hil_attitude.R[0][2] - vehicle_attitude.R[0][2]);
		fprintf (stdout, "R[1][0] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[1][0], vehicle_attitude.R[1][0], vehicle_hil_attitude.R[1][0] - vehicle_attitude.R[1][0]);
		fprintf (stdout, "R[1][1] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[1][1], vehicle_attitude.R[1][1], vehicle_hil_attitude.R[1][1] - vehicle_attitude.R[1][1]);
		fprintf (stdout, "R[1][2] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[1][2], vehicle_attitude.R[1][2], vehicle_hil_attitude.R[1][2] - vehicle_attitude.R[1][2]);
		fprintf (stdout, "R[2][0] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[2][0], vehicle_attitude.R[2][0], vehicle_hil_attitude.R[2][0] - vehicle_attitude.R[2][0]);
		fprintf (stdout, "R[2][1] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[2][1], vehicle_attitude.R[2][1], vehicle_hil_attitude.R[2][1] - vehicle_attitude.R[2][1]);
		fprintf (stdout, "R[2][2] - \t\t sim:%.5f, \test:%.5f, \tdiff:%.5f\n", vehicle_hil_attitude.R[2][2], vehicle_attitude.R[2][2], vehicle_hil_attitude.R[2][2] - vehicle_attitude.R[2][2]);

		fprintf (stdout, "\n");
		fflush (stdout);

		/* allow the user to control the console */
		release_console_control ();

		sleep(TEST_THREAD_MONITORING_INTERVAL);
	}

	/* **************************************** unsubscribe ****************************************** */
	orb_unsubscribe (ORB_ID(vehicle_attitude), vehicle_attitude_sub, pthread_self());
	orb_unsubscribe (ORB_ID(vehicle_hil_attitude), vehicle_hil_attitude_sub, pthread_self());

	return 0;
}


void* test_thread_body3 (void* args)
{
	fprintf (stderr, "\nStarting mathlib tests\n");

	Vector_test ();
	Vector2f_test ();
	Vector3f_test ();
	EulerAngles_test ();
	Quaternion_test ();
	Matrix_test ();
	Dcm_test ();

	return 0;
}
