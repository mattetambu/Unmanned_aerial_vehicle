/*
 * GUI_export.h
 *
 *  Created on: 10/ott/2013
 *      Author: Matteo
 */

#ifndef _GUI_EXPORT_H_
#define _GUI_EXPORT_H_

	#include <pthread.h>
	#include <gtk/gtk.h>
	#include "../../common.h"
	#include "../../../ORB/ORB.h"
	#include "../../../simulator/FlightGear_exchanged_data.h"
	
	#define GTK_MULTITHREAD
	#define MAX_TEXT_ROW_LENGTH				100

	/* function prototypes */
	int GUI_mission_textview_update (gpointer user_data);
	int GUI_flight_data_update ();

	/* global variables */
	extern GtkWidget *main_window;
	extern GtkWidget *properties_text_conteiners[FDM_N_PROPERTIES];
	extern GtkWidget *controls_text_conteiners[CTRL_N_CONTROLS];
	// flight data and mission textview updater
	extern guint flight_data_updater_id;
	extern guint mission_textview_updater_id;
	extern pthread_t flight_data_updater_pid;
	extern pthread_t mission_textview_updater_pid;
	// topic subscriptions
	extern orb_subscr_t GUI_mission_sub;	/* Subscription to mission topic */
	extern orb_subscr_t	GUI_mission_small_sub;	/* Subscription to mission_small topic */
	extern orb_subscr_t GUI_airspeed_sub;	/* Subscription to airspeed topic */
	extern orb_subscr_t GUI_vehicle_global_position_sub;	/* Subscription to vehicle_global_position topic */
	extern orb_subscr_t GUI_vehicle_attitude_sub;	/* Subscription to vehicle_attitude topic */
	extern orb_subscr_t GUI_actuator_controls_sub;	/* Subscription to actuator_controls topic */

#endif

