// plot.h
 
#ifndef	_PLOT_H_
#define	_PLOT_H_

	#include "common.h"
	#include "aircraft.h"
	#include "aircraft_parameters.h"

	/* function prototypes */
	void* printer_loop (void* args);
	
	/* global variables */
	extern pthread_t printer_thread_id;

#endif
