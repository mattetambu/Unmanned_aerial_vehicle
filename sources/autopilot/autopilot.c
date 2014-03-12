// autopilot.c

#include <pthread.h>

#include "autopilot.h"
#include "../uav_library/common.h"

#include "commander/commander_main.h"
#include "attitude_estimator/attitude_estimator_so3_comp_main.h"
#include "attitude_estimator/attitude_estimator_so3_comp_params.h"
#include "fw_attitude_control/fw_attitude_control_main.h"
#include "fw_attitude_control/fw_attitude_control_params.h"
#include "fw_position_control/fw_position_control_main.h"
#include "fw_position_control/fw_position_control_params.h"


pthread_t autopilot_thread_id;
int autopilot_thread_return_value = 0;


void* autopilot_loop (void* args)
{
	pthread_t commander_thread_id;
	pthread_t attitude_estimator_thread_id;
	pthread_t attitude_controller_thread_id;
	pthread_t position_controller_thread_id;


	if (commander_params_define() != 0 ||
		non_linear_SO3_AHRS_comp_params_define() != 0 ||
		fw_attitude_control_params_define() != 0 ||
		fw_position_control_params_define() != 0)
	{
		fprintf (stderr, "Autopilot thread aborting on startup due to an error\n");
		exit(-1);
	}


	// start commander
	if (pthread_create (&commander_thread_id, NULL, commander_thread_main, NULL) != 0)
	{
		fprintf (stderr, "Can't create the commander thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start attitude estimator
	if (pthread_create (&attitude_estimator_thread_id, NULL, attitude_estimator_so3_comp_thread_main, NULL) != 0)
	{
		fprintf (stderr, "Can't create the attitude estimator thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start attitude controller
	if (pthread_create (&attitude_controller_thread_id, NULL, fw_attitude_control_thread_main, NULL) != 0)
	{
		fprintf (stderr, "Can't create the attitude controller thread (errno: %d)\n", errno);
		exit(-1);
	}

	// start position controller
	if (pthread_create (&position_controller_thread_id, NULL, fw_position_control_thread_main, NULL) != 0)
	{
		fprintf (stderr, "Can't create the position controller thread (errno: %d)\n", errno);
		exit(-1);
	}


	// ************************************* wait all the modules to exit ******************************************
	pthread_join (commander_thread_id, NULL);
	pthread_join (attitude_estimator_thread_id, NULL);
	pthread_join (attitude_controller_thread_id, NULL);
	pthread_join (position_controller_thread_id, NULL);

	pthread_exit(&autopilot_thread_return_value);
	return 0;
}

