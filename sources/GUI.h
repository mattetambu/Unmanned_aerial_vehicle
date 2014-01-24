/*
 * GUI.h
 *
 *  Created on: 10/ott/2013
 *      Author: Matteo
 */

#ifndef _GUI_H_
#define _GUI_H_

	#include <glade/glade.h>
	#include <gtk/gtk.h>
	#include <pthread.h>
	#include "common.h"
	#include "autopilot.h"
	
	
	#define MAX_TEXT_ROW_LENGTH		100
	#define GUI_REFRESH_TIME		100
	

	/* function prototypes */
	void* start_GUI ();
	void close_GUI ();
	void mission_textview_update ();

	/* global variables */
	extern pthread_t GUI_thread_id;
	extern int GUI_thread_return_value;

#endif
