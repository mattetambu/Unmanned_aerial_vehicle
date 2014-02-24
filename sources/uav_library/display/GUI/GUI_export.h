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


	enum
	{
		//	SAFETY STATUS
		GUI_SAFETY_STATUS_SAFETY = 0,
		GUI_SAFETY_STATUS_ARMED,
		GUI_SAFETY_STATUS_READY_TO_ARM,
		GUI_SAFETY_STATUS_LANDED,
		GUI_SAFETY_STATUS_N_ENTRIES
	};

	enum
	{
		//	WAYPOINT ENTRIES
		GUI_HOME_WAYPOINT_LATITUDE_ENTRY = 0,
		GUI_HOME_WAYPOINT_LONGITUDE_ENTRY,
		GUI_TAKEOFF_WAYPOINT_LATITUDE_ENTRY,
		GUI_TAKEOFF_WAYPOINT_LONGITUDE_ENTRY,
		GUI_WAYPOINT_N_ENTRIES
	};

	/* function prototypes */
	int GUI_mission_textview_update (gpointer user_data);
	int GUI_flight_data_update ();

	/* global variables */
	extern GtkWidget *main_window;
	extern GtkWidget *properties_text_conteiners[FDM_N_PROPERTIES];
	extern GtkWidget *controls_text_conteiners[CTRL_N_CONTROLS];
	extern GtkWidget *safety_status_text_conteiners[GUI_SAFETY_STATUS_N_ENTRIES];
	extern GtkWidget *waypoint_entries[GUI_WAYPOINT_N_ENTRIES];
	// flight data and mission textview updater
	extern guint flight_data_updater_id;
	extern guint mission_textview_updater_id;
	extern pthread_t flight_data_updater_pid;
	extern pthread_t mission_textview_updater_pid;
	extern bool_t GUI_flight_data_updater_initialized;
	extern bool_t GUI_mission_textview_updater_initialized;
	// topic subscriptions
	extern orb_subscr_t GUI_mission_sub;	/* Subscription to mission topic */
	extern orb_subscr_t	GUI_mission_small_sub;	/* Subscription to mission_small topic */
	extern orb_subscr_t GUI_airspeed_sub;	/* Subscription to airspeed topic */
	extern orb_subscr_t GUI_vehicle_global_position_sub;	/* Subscription to vehicle_global_position topic */
	extern orb_subscr_t GUI_vehicle_attitude_sub;	/* Subscription to vehicle_attitude topic */
	extern orb_subscr_t GUI_actuator_controls_sub;	/* Subscription to actuator_controls topic */
	extern orb_subscr_t GUI_actuator_armed_sub;	/* Subscription to actuator_armed topic */
	extern orb_subscr_t GUI_safety_sub;	/* Subscription to safety topic */
	extern orb_subscr_t GUI_home_position_sub;	/* Subscription to home_position topic */
	extern orb_subscr_t GUI_takeoff_position_sub;	/* Subscription to takeoff_position topic */
	extern struct vehicle_global_position_s GUI_vehicle_global_position;

#endif

