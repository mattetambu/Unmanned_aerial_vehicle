// GUI.c

#include <glade/glade.h>
#include <gtk/gtk.h>
#include <pthread.h>

#include "GUI.h"
#include "GUI_export.h"
#include "../../common.h"
#include "../../param/param.h"
#include "../../../simulator/FlightGear_exchanged_data.h"

#include "../../../ORB/ORB.h"
#include "../../../ORB/topics/airspeed.h"
#include "../../../ORB/topics/mission.h"
#include "../../../ORB/topics/safety.h"
#include "../../../ORB/topics/vehicle_attitude.h"
#include "../../../ORB/topics/home_position.h"
#include "../../../ORB/topics/takeoff_position.h"
#include "../../../ORB/topics/manual_control_setpoint.h"
#include "../../../ORB/topics/position/vehicle_global_position.h"
#include "../../../ORB/topics/actuator/actuator_controls.h"
#include "../../../ORB/topics/actuator/actuator_armed.h"


#ifdef GTK_MULTITHREAD
	G_LOCK_DEFINE (properties_text_conteiners);
	G_LOCK_DEFINE (controls_text_conteiners);
	G_LOCK_DEFINE (safety_status_text_conteiners);
	G_LOCK_DEFINE (waypoint_entries);
#endif

enum {
	combobox_value_manual = 0,
	combobox_value_stabilize,
	combobox_value_fly_by_wire,
	combobox_value_easy,
	combobox_value_loiter,
	combobox_value_rtl,
	combobox_value_mission
};

// GUI thread
pthread_t GUI_thread_id;
int GUI_thread_return_value = 0;
bool_t first_GUI_launch = 1;

// topic advertising
orb_subscr_t GUI_safety_adv = -1;	/* safety topic */
struct safety_s safety;
pthread_t safety_controller_pid = 0;
orb_subscr_t GUI_manual_control_sp_adv = -1;	/* manual_control_setpoint topic */
struct manual_control_setpoint_s manual_control_setpoint;
pthread_t manual_control_sp_controller_pid = 0;

gint combobox_index = combobox_value_manual;



// **************************************************  GUI USER INTERACTIONS *********************************************************
void output_control_button_clicked (GtkWidget *button, gpointer user_data)
{
	if  (!getenv("DO_NOT_SEND_CONTROLS"))
		setenv ("DO_NOT_SEND_CONTROLS", "", 0);
	else
		unsetenv ("DO_NOT_SEND_CONTROLS");
}


void arm_button_clicked (GtkWidget *button, gpointer user_data)
{
	bool_t wta = (user_data == NULL)? !manual_control_setpoint.want_to_arm : *((bool_t *) user_data);
	if (GUI_manual_control_sp_adv == -1)
	{
		// ********************** Ardvertise manual_control_setpoint topics ******************************
		GUI_manual_control_sp_adv = orb_advertise (ORB_ID(manual_control_setpoint));
		if (GUI_manual_control_sp_adv < 0)
		{
			fprintf (stderr, "Failed to advertise manual_control_setpoint topic\n");
			return;
		}
		manual_control_sp_controller_pid = pthread_self();
	}

	manual_control_setpoint.want_to_arm = wta;

	orb_publish(ORB_ID(manual_control_setpoint), GUI_manual_control_sp_adv, &manual_control_setpoint);
}

void safety_button_clicked (GtkWidget *button, gpointer user_data)
{
	if (!GUI_vehicle_global_position.landed)
	{
		fprintf (stderr, "Can't activate/deactivate safety switch when MAV is not landed\n");
		return;
	}

	if (GUI_safety_adv == -1)
	{
		// ********************** Ardvertise safety topics ******************************
		GUI_safety_adv = orb_advertise (ORB_ID(safety));
		if (GUI_safety_adv < 0)
		{
			fprintf (stderr, "Failed to advertise safety topic\n");
			return;
		}
		safety_controller_pid = pthread_self();
	}

	safety.safety_off = !safety.safety_off;

	orb_publish(ORB_ID(safety), GUI_safety_adv, &safety);
}

void reset_mission_button_clicked (GtkWidget *button, gpointer user_data)
{
	// XXX NOT YET IMPLEMENTED
}



void flight_mode_combobox_changed (GtkComboBox *flight_mode_combobox, gpointer user_data)
{
	combobox_index =  gtk_combo_box_get_active(flight_mode_combobox);	// Get currently selected item's index

	//fprintf (stdout, "Selected flight_mode index: %d\n", (int) index);
	//fflush(stdout);

	/*
	// Obtain currently selected string from combo box
	gchar *string = gtk_combo_box_get_active_text (flight_mode_combobox);
	// Print it to the console - if nothing is selected, print NULL
	g_print ("Selected (simple): >> %s <<\n", (string? string : "NULL"));
	// Free string
	g_free (string);
	*/

	if (GUI_manual_control_sp_adv == -1)
	{
		// ********************** Ardvertise manual_control_setpoint topics ******************************
		GUI_manual_control_sp_adv = orb_advertise (ORB_ID(manual_control_setpoint));
		if (GUI_manual_control_sp_adv < 0)
		{
			fprintf (stderr, "Failed to advertise manual_control_setpoint topic\n");
			return;
		}
		manual_control_sp_controller_pid = pthread_self();
	}

	switch (combobox_index)
	{
		case combobox_value_manual:
			manual_control_setpoint.mode_switch = mode_switch_manual;
			manual_control_setpoint.second_switch = secondary_switch_manual;
			break;

		case combobox_value_stabilize:
			manual_control_setpoint.mode_switch = mode_switch_support;
			manual_control_setpoint.second_switch = secondary_switch_stabilize;
			break;

		case combobox_value_fly_by_wire:
			manual_control_setpoint.mode_switch = mode_switch_support;
			manual_control_setpoint.second_switch = secondary_switch_fly_by_wire;
			break;

		case combobox_value_easy:
			manual_control_setpoint.mode_switch = mode_switch_support;
			manual_control_setpoint.second_switch = secondary_switch_easy;
			break;

		case combobox_value_loiter:
			manual_control_setpoint.mode_switch = mode_switch_auto;
			manual_control_setpoint.second_switch = secondary_switch_loiter;
			break;

		case combobox_value_rtl:
			manual_control_setpoint.mode_switch = mode_switch_auto;
			manual_control_setpoint.second_switch = secondary_switch_rtl;
			break;

		case combobox_value_mission:
			manual_control_setpoint.mode_switch = mode_switch_auto;
			manual_control_setpoint.second_switch = secondary_switch_mission;
			break;

		default:
			// safety check
			// should not happen
			fprintf (stderr, "ERROR: selected combobox value not valid or not handled\n");
			break;
	}

	orb_publish(ORB_ID(manual_control_setpoint), GUI_manual_control_sp_adv, &manual_control_setpoint);
}

// **********************************************************************************************************************************


void stop_GUI (GtkWidget *button, gpointer user_data)
{
	unsetenv ("START_GUI");
	setenv ("GUI_STOPPED", "", 0);

	g_source_remove (flight_data_updater_id);
	g_source_remove (mission_textview_updater_id);

	sleep(0.5);

	// ************************************* unadvertise ******************************************
	if (GUI_safety_adv != -1 && orb_unadvertise (ORB_ID(safety), GUI_safety_adv, safety_controller_pid) < 0)
		fprintf (stderr, "Failed to unadvertise safety topic\n");

	if (GUI_manual_control_sp_adv != -1 && orb_unadvertise (ORB_ID(manual_control_setpoint), GUI_manual_control_sp_adv, manual_control_sp_controller_pid) < 0)
		fprintf (stderr, "Failed to unadvertise manual_control_setpoint topic\n");

	// ************************************* unsubscribe ******************************************
	if (orb_unsubscribe (ORB_ID(mission), GUI_mission_sub, mission_textview_updater_pid) < 0)
			fprintf (stderr, "Failed to unsubscribe to mission topic\n");

	if (orb_unsubscribe (ORB_ID(mission_small), GUI_mission_small_sub, mission_textview_updater_pid) < 0)
		fprintf (stderr, "Failed to unsubscribe to mission_small topic\n");

	if (orb_unsubscribe (ORB_ID(airspeed), GUI_airspeed_sub, flight_data_updater_pid) < 0)
		fprintf (stderr, "Failed to unsubscribe to airspeed topic\n");

	if (orb_unsubscribe (ORB_ID(vehicle_global_position), GUI_vehicle_global_position_sub, flight_data_updater_pid) < 0)
		fprintf (stderr, "Failed to unsubscribe to vehicle_global_position topic\n");

	if (orb_unsubscribe (ORB_ID(vehicle_attitude), GUI_vehicle_attitude_sub, flight_data_updater_pid) < 0)
		fprintf (stderr, "Failed to unsubscribe to vehicle_attitude topic\n");

	if (orb_unsubscribe (ORB_ID(actuator_controls), GUI_actuator_controls_sub, flight_data_updater_pid) < 0)
		fprintf (stderr, "Failed to unsubscribe to actuator_controls topic\n");

	if (orb_unsubscribe (ORB_ID(safety), GUI_safety_sub, flight_data_updater_pid) < 0)
			fprintf (stderr, "Failed to unsubscribe to safety topic\n");

	if (orb_unsubscribe (ORB_ID(actuator_armed), GUI_actuator_armed_sub, flight_data_updater_pid) < 0)
		fprintf (stderr, "Failed to unsubscribe to actuator_armed topic\n");

	if (orb_unsubscribe (ORB_ID(home_position), GUI_home_position_sub, flight_data_updater_pid) < 0)
			fprintf (stderr, "Failed to unsubscribe to home_position topic\n");

	if (orb_unsubscribe (ORB_ID(takeoff_position), GUI_takeoff_position_sub, flight_data_updater_pid) < 0)
		fprintf (stderr, "Failed to unsubscribe to takeoff_position topic\n");

	GUI_safety_adv = -1;	/* safety topic */
	GUI_manual_control_sp_adv = -1;	/* manual_control_setpoint topic */
	GUI_mission_sub = -1;	/* Subscription to mission topic */
	GUI_mission_small_sub = -1;	/* Subscription to mission_small topic */
	GUI_airspeed_sub = -1;	/* Subscription to airspeed topic */
	GUI_vehicle_global_position_sub = -1;	/* Subscription to vehicle_global_position topic */
	GUI_vehicle_attitude_sub = -1;	/* Subscription to vehicle_attitude topic */
	GUI_actuator_controls_sub = -1;	/* Subscription to actuator_controls topic */
	GUI_actuator_armed_sub = -1;	/* Subscription to actuator_armed topic */
	GUI_safety_sub = -1;	/* Subscription to safety topic */
	GUI_home_position_sub = -1;	/* Subscription to home_position topic */
	GUI_takeoff_position_sub = -1;	/* Subscription to takeoff_position topic */

	safety_controller_pid = 0;
	manual_control_sp_controller_pid = 0;
	flight_data_updater_pid = 0;
	mission_textview_updater_pid = 0;
	GUI_flight_data_updater_initialized = 0;
	GUI_mission_textview_updater_initialized = 0;

	// Exit the GTK Event Loop.
	gtk_main_quit();
}

void close_GUI ()
{
	// get GTK thread lock
	gdk_threads_enter ();

	unsetenv ("START_GUI");
	setenv ("GUI_STOPPED", "", 0);

	if (main_window)
		gtk_widget_destroy (main_window);

	// release GTK thread lock
	gdk_threads_leave ();
	
	sleep (0.2);
}

// The Xserver must be started before this function (startxwin)
void* start_GUI (void* args)
{
	int _int_zero = 0, _int_one = 1;
	GladeXML *xml;
	GtkWidget *reset_mission_button;
	GtkWidget *output_control_button;
	GtkWidget *safety_button;
	GtkWidget *arm_button;
	GtkWidget *mission_textview, *flight_mode_combobox;
	
	//Set DISPLAY enviroment variable if not set
	setenv ("DISPLAY", ":0", 0);

	// get GTK thread lock
	gdk_threads_enter ();
 
	//Initialize GTK+.
	if (!gtk_init_check(0, NULL))
	{
		fprintf (stderr, "\nFailed to initialize gtk\n");
		fprintf (stderr, "The Xserver must be started before using the GUI (startxwin)\n");
		fprintf (stderr, "Set very-verbose flag (from command-line) to see exanched data\n\n");
		setenv ("GUI_INIT_ERROR", "", 0);
		unsetenv ("START_GUI");
		GUI_thread_return_value = -1;
		pthread_exit(&GUI_thread_return_value);
	}

	//Load the interface description.
	xml = glade_xml_new("uav_library/display/GUI/UAV_autopilot_GUI.glade", NULL, NULL);
	if (!xml)
	{
		fprintf (stderr, "Failed to load the GUI from glade xml\n");
		GUI_thread_return_value = -1;
		pthread_exit(&GUI_thread_return_value);
	}

	//glade_xml_signal_autoconnect(xml);
	
	// lock the properties_text_conteiners variable
#ifdef GTK_MULTITHREAD
	G_LOCK (properties_text_conteiners);
#endif
		
	//Find the main window (not shown by default, ogcalcmm.cc need sit to be hidden initially) and then show it.
	main_window = glade_xml_get_widget (xml,"UAV_autopilot_GUI_main_window");
	properties_text_conteiners[FDM_FLIGHT_TIME] = glade_xml_get_widget (xml,"Time_value");
	properties_text_conteiners[FDM_LATITUDE] = glade_xml_get_widget (xml,"Latitude_value");
	properties_text_conteiners[FDM_LONGITUDE] = glade_xml_get_widget (xml,"Longitude_value");
	properties_text_conteiners[FDM_ALTITUDE] = glade_xml_get_widget (xml,"Altitude_value");
	properties_text_conteiners[FDM_GROUND_LEVEL] = glade_xml_get_widget (xml,"Ground_level_value");
	properties_text_conteiners[FDM_ROLL_ANGLE] = glade_xml_get_widget (xml,"Roll_angle_value");
	properties_text_conteiners[FDM_PITCH_ANGLE] = glade_xml_get_widget (xml,"Pitch_angle_value");
	properties_text_conteiners[FDM_YAW_ANGLE] = glade_xml_get_widget (xml,"Yaw_angle_value");
	properties_text_conteiners[FDM_ROLL_RATE] = glade_xml_get_widget (xml,"Roll_rate_value");
	properties_text_conteiners[FDM_PITCH_RATE] = glade_xml_get_widget (xml,"Pitch_rate_value");
	properties_text_conteiners[FDM_YAW_RATE] = glade_xml_get_widget (xml,"Yaw_rate_value");
	properties_text_conteiners[FDM_X_BODY_VELOCITY] = glade_xml_get_widget (xml,"X_body_velocity_value");
	properties_text_conteiners[FDM_Y_BODY_VELOCITY] = glade_xml_get_widget (xml,"Y_body_velocity_value");
	properties_text_conteiners[FDM_Z_BODY_VELOCITY] = glade_xml_get_widget (xml,"Z_body_velocity_value");
	properties_text_conteiners[FDM_X_EARTH_VELOCITY] = glade_xml_get_widget (xml,"X_earth_velocity_value");
	properties_text_conteiners[FDM_Y_EARTH_VELOCITY] = glade_xml_get_widget (xml,"Y_earth_velocity_value");
	properties_text_conteiners[FDM_Z_EARTH_VELOCITY] = glade_xml_get_widget (xml,"Z_earth_velocity_value");
	properties_text_conteiners[FDM_AIRSPEED] = glade_xml_get_widget (xml,"Airspeed_value");
	properties_text_conteiners[FDM_X_BODY_ACCELERATION] = glade_xml_get_widget (xml,"X_body_acceleration_value");
	properties_text_conteiners[FDM_Y_BODY_ACCELERATION] = glade_xml_get_widget (xml,"Y_body_acceleration_value");
	properties_text_conteiners[FDM_Z_BODY_ACCELERATION] = glade_xml_get_widget (xml,"Z_body_acceleration_value");
	properties_text_conteiners[FDM_ENGINE_ROTATION_SPEED] = glade_xml_get_widget (xml,"Engine_rotation_speed_value");
	properties_text_conteiners[FDM_ENGINE_THRUST] = glade_xml_get_widget (xml,"Engine_thrust_value");

#ifdef GTK_MULTITHREAD	
	// unlock the properties_text_conteiners variable
	G_UNLOCK (properties_text_conteiners);
	
	// lock the controls_text_conteiners variable
	G_LOCK (controls_text_conteiners);
#endif

	controls_text_conteiners[CTRL_AILERON] = glade_xml_get_widget (xml,"Aileron_value");
	controls_text_conteiners[CTRL_ELEVATOR] = glade_xml_get_widget (xml,"Elevator_value");
	controls_text_conteiners[CTRL_RUDDER] = glade_xml_get_widget (xml,"Rudder_value");
	controls_text_conteiners[CTRL_THROTTLE] = glade_xml_get_widget (xml,"Throttle_value");

#ifdef GTK_MULTITHREAD
	// unlock the controls_text_conteiners variable
	G_UNLOCK (controls_text_conteiners);

	// lock the safety_status_text_conteiners variable
	G_LOCK (safety_status_text_conteiners);
#endif

	safety_status_text_conteiners[GUI_SAFETY_STATUS_SAFETY] = glade_xml_get_widget (xml,"Safety_value");
	safety_status_text_conteiners[GUI_SAFETY_STATUS_ARMED] = glade_xml_get_widget (xml,"Armed_value");
	safety_status_text_conteiners[GUI_SAFETY_STATUS_READY_TO_ARM] = glade_xml_get_widget (xml,"Ready_to_arm_value");
	safety_status_text_conteiners[GUI_SAFETY_STATUS_LANDED] = glade_xml_get_widget (xml,"Landed_value");

#ifdef GTK_MULTITHREAD
	// unlock the safety_status_text_conteiners variable
	G_UNLOCK (safety_status_text_conteiners);

	// lock the waypoint_entries variable
	G_LOCK (waypoint_entries);
#endif

	waypoint_entries[GUI_HOME_WAYPOINT_LATITUDE_ENTRY] = glade_xml_get_widget (xml, "Home_waypoint_latitude_entry");
	waypoint_entries[GUI_HOME_WAYPOINT_LONGITUDE_ENTRY] =  glade_xml_get_widget (xml, "Home_waypoint_longitude_entry");
	waypoint_entries[GUI_TAKEOFF_WAYPOINT_LONGITUDE_ENTRY] = glade_xml_get_widget (xml, "Takeoff_waypoint_longitude_entry");
	waypoint_entries[GUI_TAKEOFF_WAYPOINT_LATITUDE_ENTRY] = glade_xml_get_widget (xml, "Takeoff_waypoint_latitude_entry");

#ifdef GTK_MULTITHREAD
	// unlock the waypoint_entries variable
	G_UNLOCK (waypoint_entries);
#endif


	output_control_button = glade_xml_get_widget (xml, "Output_control_button");
	arm_button = glade_xml_get_widget (xml, "Arm_button");
	safety_button = glade_xml_get_widget (xml, "Safety_button");
	reset_mission_button = glade_xml_get_widget (xml, "Reset_mission_button");
	flight_mode_combobox = glade_xml_get_widget (xml, "Flight_mode_combobox");
	mission_textview = glade_xml_get_widget (xml, "Mission_textview");
	
	//Setup the signal handlers
	glade_xml_signal_connect (xml, "on_window_destroy", G_CALLBACK (gtk_main_quit));
	g_signal_connect (G_OBJECT (main_window), "destroy", G_CALLBACK (stop_GUI), NULL);
	g_signal_connect (G_OBJECT (output_control_button), "clicked", G_CALLBACK (output_control_button_clicked), NULL);
	//g_signal_connect (G_OBJECT (arm_button), "clicked", G_CALLBACK (arm_button_clicked), NULL);
	g_signal_connect (G_OBJECT (arm_button), "pressed", G_CALLBACK (arm_button_clicked), &_int_one);
	g_signal_connect (G_OBJECT (arm_button), "released", G_CALLBACK (arm_button_clicked), &_int_zero);
	g_signal_connect (G_OBJECT (safety_button), "clicked", G_CALLBACK (safety_button_clicked), NULL);
	g_signal_connect (G_OBJECT (reset_mission_button), "clicked", G_CALLBACK (reset_mission_button_clicked), NULL);
	g_signal_connect (G_OBJECT (flight_mode_combobox), "changed", G_CALLBACK (flight_mode_combobox_changed), NULL);


	// ********************** Publish one time the safety topics ******************************
	if (first_GUI_launch)
	{
		GUI_safety_adv = orb_advertise (ORB_ID(safety));
		if (GUI_safety_adv < 0)
		{
			fprintf (stderr, "Failed to advertise safety topic\n");
		}
		else
		{
			safety.safety_switch_available = 1;
			safety.safety_off = 0; // start in safety mode
			orb_publish(ORB_ID(safety), GUI_safety_adv, &safety);
			orb_unadvertise (ORB_ID(safety), GUI_safety_adv, pthread_self());
		}

		memset (&manual_control_setpoint, 0, sizeof(manual_control_setpoint));
		GUI_safety_adv = -1;
		first_GUI_launch = 0;
	}

	//Setup the interface status
	if (flight_mode_combobox) gtk_combo_box_set_active ((GtkComboBox *) flight_mode_combobox, combobox_index /* manual control at first */);

	unsetenv ("GUI_STOPPED");
	
	// Periodic tasks
	// start the periodic export of flight data and flight controls
	flight_data_updater_id = g_timeout_add (GUI_REFRESH_TIME, GUI_flight_data_update, NULL);
	// start the periodic update of mission text view
	mission_textview_updater_id = g_timeout_add (MISSION_TEXTVIEW_REFRESH_TIME, GUI_mission_textview_update, (gpointer) mission_textview);

	//Enter the GTK Event Loop. This is where all the events are caught and handled. It is exited with gtk main quit().
	gtk_widget_show (main_window);
	gtk_main();

	// release GTK thread lock
	gdk_threads_leave ();
	
	pthread_exit(&GUI_thread_return_value);
	return 0;
}
