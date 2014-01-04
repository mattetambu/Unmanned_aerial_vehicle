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
	#include "autopilot_parameters.h"
	
	#define MAX_TEXT_ROW_LENGTH 100

	/* function prototypes */
	void* start_GUI ();

	/* global variables */
	extern GtkWidget* properties_text_conteiners[N_PROPERTIES];
	extern GtkWidget* controls_text_conteiners[N_CONTROLS];
	extern int GUI_thread_return_value;

#endif
