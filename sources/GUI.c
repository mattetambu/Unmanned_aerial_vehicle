// GUI.c

#include "GUI.h"
#include "common.h"
#include "mission_logic.h"
#include "autopilot_logic.h"
#include "autopilot_parameters.h"
#include "plot.h"

GtkWidget* properties_text_conteiners[N_PROPERTIES];
GtkWidget* controls_text_conteiners[N_CONTROLS];
int GUI_thread_return_value = 0;

void output_control_button_clicked (GtkWidget *button, gpointer user_data)
{
	int i;
	if  (!getenv("DO_NOT_SEND_CONTROLS"))
	{
		setenv ("DO_NOT_SEND_CONTROLS", "", 0);
		sleep(0.1);
		for (i = 0; i < N_CONTROLS; i++)
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[i]), "-");
	}
	else
		unsetenv ("DO_NOT_SEND_CONTROLS");
}

void reset_mission_button_clicked (GtkWidget *button, gpointer user_data)
{
	fprintf (stdout, "Mission restarted\n");
	fflush (stdout);
}

void textview_buffer_fill_whit_string (GtkTextBuffer *buffer, gchar *text_row, int text_length)
{
	GtkTextMark *mark;
	GtkTextIter iter;
	
	mark = gtk_text_buffer_get_insert (buffer);
	gtk_text_buffer_get_iter_at_mark (buffer, &iter, mark);
	gtk_text_buffer_insert (buffer, &iter, text_row, text_length);	
}

int mission_textview_fill (GtkTextView * mission_textview, gpointer user_data) {
	mission_command_t *command;
	gchar text_row [MAX_TEXT_ROW_LENGTH];
	int text_length;
	GtkTextBuffer *buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (mission_textview));
	
	text_length = snprintf ((char *) text_row, MAX_TEXT_ROW_LENGTH, "* mission mode=\"%s\" lastly=\"%s\"\n",
						mission_mode_to_string(mission->mode),
						mission_lastly_cmd_to_string(mission->lastly));
	textview_buffer_fill_whit_string (buffer, text_row, text_length);

	for (command = mission->command_list; command; command = command->next)
	{
		text_length = plot_mission_command (command, (char*) text_row, MAX_TEXT_ROW_LENGTH);
		if (text_length < 0)
		{
			fprintf (stderr , "Failed to plot a mission command\n");
			return -1;
		}
		textview_buffer_fill_whit_string (buffer, text_row, text_length);
	}

	text_length = snprintf ((char *) text_row, MAX_TEXT_ROW_LENGTH, "  mission end");
	textview_buffer_fill_whit_string (buffer, text_row, text_length);

	return 0;
}

void fly_to_waypoint_latitude_changed (GtkEntry *entry, gpointer user_data)
{
	// Obtain text string from entry
	const gchar *text = gtk_entry_get_text (entry);

	// Print it to the console - if nothing is selected, print NULL
	fprintf (stdout, "Fly_to waypoint latitude: %s\n", (text? text : "NULL"));
	fflush (stdout);
}

void fly_to_waypoint_longitude_changed (GtkEntry *entry, gpointer user_data)
{
	// Obtain text string from entry
	const gchar *text = gtk_entry_get_text (entry);

	// Print it to the console - if nothing is selected, print NULL
	fprintf (stdout, "Fly_to waypoint longitude: %s\n", (text? text : "NULL"));
	fflush (stdout);
}


void flight_mode_combobox_changed (GtkComboBox *flight_mode_combobox, gpointer user_data)
{
	gint index;

	// Get currently selected item's index
	index = gtk_combo_box_get_active(flight_mode_combobox);
	aircraft->flight_mode = (flight_mode_t) index;

	fprintf (stdout, "Selected flight_mode index: %d\n", (int) aircraft->flight_mode);
	fflush(stdout);

	/*
	// Obtain currently selected string from combo box
	gchar *string = gtk_combo_box_get_active_text (flight_mode_combobox);
	// Print it to the console - if nothing is selected, print NULL
	g_print ("Selected (simple): >> %s <<\n", (string? string : "NULL"));
	// Free string
	g_free (string);
	*/
}

void stop_GUI (GtkWidget *button, gpointer user_data)
{
	unsetenv ("START_GUI");
	setenv ("GUI_STOPPED", "", 0);
}


// The Xserver must be started before this function (startxwin)
void* start_GUI (void* args)
{
	GladeXML *xml;
	GtkWidget *main_window;
	GtkWidget *output_control_button, *reset_mission_button;
	GtkWidget *fly_to_waypoint_latitude_entry, *fly_to_waypoint_longitude_entry;
	//GtkWidget *home_waypoint_latitude_entry, *home_waypoint_longitude_entry;
	GtkWidget *mission_textview, *flight_mode_combobox;
	
	//Set DISPLAY enviroment variable if not set
	setenv ("DISPLAY", ":0", 0);

	//Initialize GTK+.
	if (!gtk_init_check(0,NULL))
	{
		fprintf (stderr, "Failed to initialize gtk\n");
		GUI_thread_return_value = -1;
		pthread_exit(&GUI_thread_return_value);
	}

	//Load the interface description.
	xml = glade_xml_new("FlightGear_comunicator_GUI.glade", NULL, NULL);
	if (!xml)
	{
		fprintf (stderr, "Failed to load glade xml\n");
		GUI_thread_return_value = -1;
		pthread_exit(&GUI_thread_return_value);
	}

	//glade_xml_signal_autoconnect(xml);
	
	//Find the main window (not shown by default, ogcalcmm.cc need sit to be hidden initially) and then show it.
	main_window = glade_xml_get_widget (xml,"FlightGear_comunicator_main_window");
	properties_text_conteiners[FLIGHT_TIME] = glade_xml_get_widget (xml,"Time_value");
	properties_text_conteiners[LATITUDE] = glade_xml_get_widget (xml,"Latitude_value");
	properties_text_conteiners[LONGITUDE] = glade_xml_get_widget (xml,"Longitude_value");
	properties_text_conteiners[ALTITIUDE] = glade_xml_get_widget (xml,"Altitude_value");
	properties_text_conteiners[GROUND_LEVEL] = glade_xml_get_widget (xml,"Ground_level_value");
	properties_text_conteiners[ROLL_ANGLE] = glade_xml_get_widget (xml,"Roll_angle_value");
	properties_text_conteiners[PITCH_ANGLE] = glade_xml_get_widget (xml,"Pitch_angle_value");
	properties_text_conteiners[YAW_ANGLE] = glade_xml_get_widget (xml,"Yaw_angle_value");
	properties_text_conteiners[ROLL_RATE] = glade_xml_get_widget (xml,"Roll_rate_value");
	properties_text_conteiners[PITCH_RATE] = glade_xml_get_widget (xml,"Pitch_rate_value");
	properties_text_conteiners[YAW_RATE] = glade_xml_get_widget (xml,"Yaw_rate_value");
	properties_text_conteiners[U_VELOCITY] = glade_xml_get_widget (xml,"U_velocity_value");
	properties_text_conteiners[V_VELOCITY] = glade_xml_get_widget (xml,"V_velocity_value");
	properties_text_conteiners[W_VELOCITY] = glade_xml_get_widget (xml,"W_velocity_value");
	properties_text_conteiners[NORTH_VELOCITY] = glade_xml_get_widget (xml,"North_velocity_value");
	properties_text_conteiners[EAST_VELOCITY] = glade_xml_get_widget (xml,"East_velocity_value");
	properties_text_conteiners[DOWN_VELOCITY] = glade_xml_get_widget (xml,"Down_velocity_value");
	properties_text_conteiners[AIRSPEED] = glade_xml_get_widget (xml,"Airspeed_value");
	properties_text_conteiners[X_ACCELERATION] = glade_xml_get_widget (xml,"X_acceleration_value");
	properties_text_conteiners[Y_ACCELERATION] = glade_xml_get_widget (xml,"Y_acceleration_value");
	properties_text_conteiners[Z_ACCELERATION] = glade_xml_get_widget (xml,"Z_acceleration_value");
	properties_text_conteiners[NORTH_ACCELERATION] = glade_xml_get_widget (xml,"North_acceleration_value");
	properties_text_conteiners[EAST_ACCELERATION] = glade_xml_get_widget (xml,"East_acceleration_value");
	properties_text_conteiners[DOWN_ACCELERATION] = glade_xml_get_widget (xml,"Down_acceleration_value");
	properties_text_conteiners[ENGINE_RPM] = glade_xml_get_widget (xml,"Engine_rpm_value");
	
	controls_text_conteiners[AILERON] = glade_xml_get_widget (xml,"Aileron_value");
	controls_text_conteiners[ELEVATOR] = glade_xml_get_widget (xml,"Elevator_value");
	controls_text_conteiners[RUDDER] = glade_xml_get_widget (xml,"Rudder_value");
	controls_text_conteiners[THROTTLE] = glade_xml_get_widget (xml,"Throttle_value");

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

	//Setup the interface status
	if (flight_mode_combobox) gtk_combo_box_set_active ((GtkComboBox *) flight_mode_combobox, 0);
	if (mission_textview && mission_textview_fill ((GtkTextView *) mission_textview, 0) < 0)
	{
		fprintf (stderr, "Failed to fill the GUI mission textbox\n");
		GUI_thread_return_value = -1;
		pthread_exit(&GUI_thread_return_value);
	}
	
	//Enter the GTK Event Loop. This is where all the events are caught and handled. It is exited with gtk main quit().
	gtk_widget_show (main_window);
	gtk_main();

	pthread_exit(&GUI_thread_return_value);
	return 0;
}
