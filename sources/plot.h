// plot.h
 
#ifndef	_PLOT_H_
#define	_PLOT_H_

	#include "common.h"
	#include "mission_logic.h"

	#define N_SPACES_PER_TAB	5

	/* function prototypes */
	int plot_flight_data (double* flight_data);
	int plot_flight_controls (double* flight_controls);
	int plot_mission_command (mission_command_t *command, char *buffer_ptr, int text_length);
	
	/* global variables */
	//empty

#endif
