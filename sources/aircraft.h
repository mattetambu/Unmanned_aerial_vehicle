// aircraft.h

#ifndef	_AIRCRAFT_H_
#define	_AIRCRAFT_H_

	#include "common.h"
	#include "aircraft_parameters.h"
	
	
	typedef	struct aircraft_t
	{
		double flight_data [N_PROPERTIES];		// FDM data
		double flight_controls [N_CONTROLS];	// Flight Controls
	} aircraft_t;

	
	/* function prototypes */
	int aircraft_init ();
	void aircraft_destroy ();
	
	/* global variables */
	extern aircraft_t *aircraft;

#endif
