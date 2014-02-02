/*
 * GUI.h
 *
 *  Created on: 10/ott/2013
 *      Author: Matteo
 */

#ifndef _GUI_H_
#define _GUI_H_

	#include <pthread.h>
	#include "../../common.h"
	
	#define GUI_REFRESH_TIME				200
	#define MISSION_TEXTVIEW_REFRESH_TIME	500

	/* function prototypes */
	void* start_GUI ();
	void close_GUI ();

	/* global variables */
	extern pthread_t GUI_thread_id;
	extern int GUI_thread_return_value;

#endif
