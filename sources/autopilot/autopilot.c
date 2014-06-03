// autopilot.c

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>

#include "autopilot.h"
#include "../uav_type.h"
#include "../uav_library/common.h"

#include "commander/commander_main.h"
#include "mixer/mixer_main.h"
#include "attitude_estimator_SO3/attitude_estimator_so3_comp_main.h"
#include "attitude_estimator_SO3/attitude_estimator_so3_comp_params.h"
#include "attitude_estimator_ideal/attitude_estimator_ideal.h"
#include "position_estimator_mc/position_estimator_mc_main.h"
#include "position_estimator_mc/position_estimator_mc_params.h"
#include "position_estimator_ideal/position_estimator_ideal_main.h"
#include "fw_attitude_control/fw_attitude_control_main.h"
#include "fw_attitude_control/fw_attitude_control_params.h"
#include "fw_position_control/fw_position_control_main.h"
#include "fw_position_control/fw_position_control_params.h"
#include "fixedwing_position_control/fixedwing_position_control_main.h"
#include "fixedwing_position_control/fixedwing_position_control_params.h"
#include "fixedwing_attitude_control/fixedwing_attitude_control_main.h"
#include "fixedwing_attitude_control/fixedwing_attitude_control_rate_params.h"
#include "fixedwing_attitude_control/fixedwing_attitude_control_att_params.h"
#include "multirotor_position_control/multirotor_position_control_main.h"
#include "multirotor_position_control/multirotor_position_control_params.h"
#include "multirotor_attitude_control/multirotor_attitude_control_main.h"
#include "multirotor_attitude_control/multirotor_rate_control_params.h"
#include "multirotor_attitude_control/multirotor_attitude_control_params.h"
#include "mr_position_control/mr_position_control_main.h"
#include "mr_position_control/mr_position_control_params.h"
#include "mr_attitude_control/mr_attitude_control_main.h"
#include "mr_attitude_control/mr_attitude_control_params.h"

#include "../mission/mission_waypoint_manager.h"
#include "../mission/mission_waypoint_manager_params.h"


#define USE_IDEAL_ATTITUDE_ESTIMATION
#define USE_IDEAL_POSITION_ESTIMATION
#define USE_ECL_L1_FW_CONTROL			// comment it to use the other control algorithm for fw (not recommended)
//#define USE_NEW_MULTIROTOR_CONTROL	// uncomment it to use the other control algorithm for mr (not recommended)

pthread_t autopilot_thread_id;
int autopilot_thread_return_value = 0;


void* autopilot_loop (void* args)
{
	pthread_t commander_thread_id;
	pthread_t attitude_estimator_thread_id;
	pthread_t position_estimator_thread_id;
	pthread_t attitude_controller_thread_id;
	pthread_t position_controller_thread_id;
	pthread_t mixer_thread_id;
	pthread_t wp_manager_thread_id;


	if (
		commander_params_define() != 0 ||
		mission_waypoint_manager_params_define() != 0 ||
#ifdef USE_IDEAL_ATTITUDE_ESTIMATION
		0 ||
#else
		non_linear_SO3_AHRS_comp_params_define() != 0 ||
#endif

#ifdef USE_IDEAL_POSITION_ESTIMATION
		0
#else
		position_estimator_mc_params_define() != 0
#endif
		)
	{
		fprintf (stderr, "Autopilot thread aborting on startup due to an error\n");
		exit(-1);
	}

	if (!is_rotary_wing &&
		(
#ifdef USE_ECL_L1_FW_CONTROL
		fw_attitude_control_params_define() != 0 ||
		fw_position_control_params_define() != 0
#else
		fixedwing_attitude_control_rate_params_define() != 0 ||
		fixedwing_attitude_control_att_params_define() != 0 ||
		fixedwing_position_control_params_define() != 0
#endif
		))
	{
		fprintf (stderr, "Autopilot thread aborting on startup due to an error\n");
		exit(-1);
	}

	if (is_rotary_wing &&
		(
#ifndef USE_NEW_MULTIROTOR_CONTROL
		multirotor_rate_control_params_define() != 0 ||
		multirotor_attitude_control_params_define() != 0 ||
		multirotor_position_control_params_define() != 0
#else
		mr_attitude_control_params_define() != 0 ||
		mr_position_control_params_define() != 0
#endif
		))
	{
		fprintf (stderr, "Autopilot thread aborting on startup due to an error\n");
		exit(-1);
	}

	// do not remove
	sleep (5);

	// start commander
	if (pthread_create (&commander_thread_id, NULL, commander_thread_main, NULL) != 0)
	{
		fprintf (stderr, "Can't create the commander thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start attitude estimator
#ifdef USE_IDEAL_ATTITUDE_ESTIMATION
	if (pthread_create (&attitude_estimator_thread_id, NULL, attitude_estimator_ideal_thread_main, NULL) != 0)
#else
	if (pthread_create (&attitude_estimator_thread_id, NULL, attitude_estimator_so3_comp_thread_main, NULL) != 0)
#endif
	{
		fprintf (stderr, "Can't create the attitude estimator thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start position estimator
#ifdef USE_IDEAL_POSITION_ESTIMATION
	if (pthread_create (&position_estimator_thread_id, NULL, position_estimator_ideal_thread_main, NULL) != 0)
#else
	if (pthread_create (&position_estimator_thread_id, NULL, position_estimator_mc_thread_main, NULL) != 0)
#endif
	{
		fprintf (stderr, "Can't create the attitude estimator thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start attitude controller
	if (!is_rotary_wing &&
#ifdef USE_ECL_L1_FW_CONTROL
		pthread_create (&attitude_controller_thread_id, NULL, fw_attitude_control_thread_main, NULL) != 0)
#else
		pthread_create (&attitude_controller_thread_id, NULL, fixedwing_attitude_control_thread_main, NULL) != 0)
#endif
	{
		fprintf (stderr, "Can't create the attitude controller thread (errno: %d)\n", errno);
		exit(-1);
	}
	else if (is_rotary_wing &&
#ifndef USE_NEW_MULTIROTOR_CONTROL
		pthread_create (&position_controller_thread_id, NULL, multirotor_attitude_control_thread_main, NULL) != 0)
#else
		pthread_create (&position_controller_thread_id, NULL, (void *) mr_attitude_control_thread_main, NULL) != 0)
#endif
	{
		fprintf (stderr, "Can't create the attitude controller thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start position controller
	if (!is_rotary_wing &&
#ifdef USE_ECL_L1_FW_CONTROL
		pthread_create (&position_controller_thread_id, NULL, fw_position_control_thread_main, NULL) != 0)
#else
		pthread_create (&position_controller_thread_id, NULL, fixedwing_position_control_thread_main, NULL) != 0)
#endif
	{
		fprintf (stderr, "Can't create the position controller thread (errno: %d)\n", errno);
		exit(-1);
	}
	else if (is_rotary_wing &&
#ifndef USE_NEW_MULTIROTOR_CONTROL
		pthread_create (&position_controller_thread_id, NULL, multirotor_position_control_thread_main, NULL) != 0)
#else
		pthread_create (&position_controller_thread_id, NULL, (void *) mr_position_control_thread_main, NULL) != 0)
#endif
	{
		fprintf (stderr, "Can't create the position controller thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start waypoint manager
	if (pthread_create (&wp_manager_thread_id, NULL, mission_waypoint_manager_thread_main, NULL) != 0)
	{
		fprintf (stderr, "Can't create the waypoint manager thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start mixer
	if (pthread_create (&mixer_thread_id, NULL, mixer_thread_main, NULL) != 0)
	{
		fprintf (stderr, "Can't create the mixer thread (errno: %d)\n", errno);
		exit(-1);
	}

	fprintf (stdout, "\n\n");
	fflush(stdout);


	// ************************************* wait all the modules to exit ******************************************
	pthread_join (commander_thread_id, NULL);
	pthread_join (attitude_estimator_thread_id, NULL);
	pthread_join (position_estimator_thread_id, NULL);
	pthread_join (attitude_controller_thread_id, NULL);
	pthread_join (position_controller_thread_id, NULL);
	pthread_join (wp_manager_thread_id, NULL);
	pthread_join (mixer_thread_id, NULL);

	pthread_exit(&autopilot_thread_return_value);
	return 0;
}

