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
	#include "common.h"
	#include "autopilot.h"
	#include "plot.h"
	
	
	#define MAX_TEXT_ROW_LENGTH		100
	#define GUI_REFRESH_TIME		100
	

	/* function prototypes */
	void* start_GUI ();

	/* global variables */
	extern int GUI_thread_return_value;

#endif
