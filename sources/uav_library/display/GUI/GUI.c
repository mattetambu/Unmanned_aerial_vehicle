// GUI.c

#include <glade/glade.h>
#include <gtk/gtk.h>
#include <pthread.h>

#include "GUI.h"
#include "GUI_export.h"
#include "../../common.h"
#include "../../../simulator/FlightGear_exchanged_data.h"

#include "../../../ORB/ORB.h"
#include "../../../ORB/topics/airspeed.h"
#include "../../../ORB/topics/vehicle_attitude.h"
#include "../../../ORB/topics/vehicle_global_position.h"
#include "../../../ORB/topics/actuator/actuator_controls.h"
#include "../../../ORB/topics/mission.h"


#ifdef GTK_MULTITHREAD
	G_LOCK_DEFINE (properties_text_conteiners);
	G_LOCK_DEFINE (controls_text_conteiners);
#endif


// GUI thread
pthread_t GUI_thread_id;
int GUI_thread_return_value = 0;



// **************************************************  GUI USER INTERACTIONS *********************************************************
void output_control_button_clicked (GtkWidget *button, gpointer user_data)
{
	if  (!getenv("DO_NOT_SEND_CONTROLS"))
		setenv ("DO_NOT_SEND_CONTROLS", "", 0);
	else
		unsetenv ("DO_NOT_SEND_CONTROLS");
}

void reset_mission_button_clicked (GtkWidget *button, gpointer user_data)
{
	// xxx NOT YET IMPLEMENTED
}

void fly_to_waypoint_latitude_changed (GtkEntry *entry, gpointer user_data)
{
	// NOT YET IMPLEMENTED
	// Obtain text string from entry
	const gchar *text = gtk_entry_get_text (entry);

	// Print it to the console - if nothing is selected, print NULL
	fprintf (stdout, "Fly_to waypoint latitude: %s\n", (text? text : "NULL"));
	fflush (stdout);
}

void fly_to_waypoint_longitude_changed (GtkEntry *entry, gpointer user_data)
{
	// NOT YET IMPLEMENTED
	// Obtain text string from entry
	const gchar *text = gtk_entry_get_text (entry);

	// Print it to the console - if nothing is selected, print NULL
	fprintf (stdout, "Fly_to waypoint longitude: %s\n", (text? text : "NULL"));
	fflush (stdout);
}

void flight_mode_combobox_changed (GtkComboBox *flight_mode_combobox, gpointer user_data)
{
	// NOT YET IMPLEMENTED
	//gint index =  gtk_combo_box_get_active(flight_mode_combobox);	// Get currently selected item's index

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
}
// **********************************************************************************************************************************


void stop_GUI (GtkWidget *button, gpointer user_data)
{
	unsetenv ("START_GUI");
	setenv ("GUI_STOPPED", "", 0);

	g_source_remove (flight_data_updater_id);
	g_source_remove (mission_textview_updater_id);

	sleep(0.5);

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

	GUI_mission_sub = -1;	/* Subscription to mission topic */
	GUI_mission_small_sub = -1;	/* Subscription to mission_small topic */
	GUI_airspeed_sub = -1;	/* Subscription to airspeed topic */
	GUI_vehicle_global_position_sub = -1;	/* Subscription to vehicle_global_position topic */
	GUI_vehicle_attitude_sub = -1;	/* Subscription to vehicle_attitude topic */
	GUI_actuator_controls_sub = -1;	/* Subscription to actuator_controls topic */

	flight_data_updater_pid = 0;
	mission_textview_updater_pid = 0;

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
	GladeXML *xml;
	GtkWidget *output_control_button, *reset_mission_button;
	GtkWidget *fly_to_waypoint_latitude_entry, *fly_to_waypoint_longitude_entry;
	//GtkWidget *home_waypoint_latitude_entry, *home_waypoint_longitude_entry;
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
#endif

	output_control_button = glade_xml_get_widget (xml, "Output_control_button");
	reset_mission_button = glade_xml_get_widget (xml, "Reset_mission_button");
	flight_mode_combobox = glade_xml_get_widget (xml, "Flight_mode_combobox");
	mission_textview = glade_xml_get_widget (xml, "Mission_textview");
	
	fly_to_waypoint_latitude_entry = glade_xml_get_widget (xml, "Fly_to_waypoint_latitude_entry");
	fly_to_waypoint_longitude_entry = glade_xml_get_widget (xml, "Fly_to_waypoint_longitude_entry");
	//home_waypoint_latitude_entry = glade_xml_get_widget (xml, "Home_waypoint_latitude_entry");
	//home_waypoint_longitude_entry =	 glade_xml_get_widget (xml, "Home_waypoint_longitude_entry");

	//Setup the signal handlers
	glade_xml_signal_connect (xml, "on_window_destroy", G_CALLBACK (gtk_main_quit));
	g_signal_connect (G_OBJECT (main_window), "destroy", G_CALLBACK (stop_GUI), NULL);
	g_signal_connect (G_OBJECT (output_control_button), "clicked", G_CALLBACK (output_control_button_clicked), NULL);
	g_signal_connect (G_OBJECT (reset_mission_button), "clicked", G_CALLBACK (reset_mission_button_clicked), NULL);
	g_signal_connect (G_OBJECT (flight_mode_combobox), "changed", G_CALLBACK (flight_mode_combobox_changed), NULL);
	g_signal_connect (G_OBJECT (fly_to_waypoint_latitude_entry), "changed", G_CALLBACK (fly_to_waypoint_latitude_changed), NULL);
	g_signal_connect (G_OBJECT (fly_to_waypoint_longitude_entry), "changed", G_CALLBACK (fly_to_waypoint_longitude_changed), NULL);


	unsetenv ("GUI_STOPPED");

	//Setup the interface status
	if (flight_mode_combobox) gtk_combo_box_set_active ((GtkComboBox *) flight_mode_combobox, 0 /* manual control */);
	
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
