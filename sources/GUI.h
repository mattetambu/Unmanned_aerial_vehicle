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

	/* function prototypes */
	void* start_GUI ();

	/* global variables */
	extern GtkWidget* properties_text_conteiners[N_USED_PROPERTIES];
	extern GtkWidget* controls_text_conteiners[N_USED_CONTROLS];

#endif
