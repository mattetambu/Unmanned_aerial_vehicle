// GUI.c

#include "GUI.h"
#include "common.h"

GtkWidget* properties_text_conteiners[N_USED_PROPERTIES];
GtkWidget* controls_text_conteiners[N_USED_CONTROLS];


void control_button_clicked (GtkWidget *button, gpointer user_data)
{
	int i;
	if  (!getenv("DO_NOT_SEND_CONTROLS"))
	{
		for (i = 0; i < N_USED_CONTROLS; i++)
				gtk_label_set_text (GTK_LABEL (controls_text_conteiners[i]), "-");
		setenv ("DO_NOT_SEND_CONTROLS", "", 0);
	}
	else
		unsetenv ("DO_NOT_SEND_CONTROLS");
}

void stop_GUI (GtkWidget *button, gpointer user_data)
{
	unsetenv ("START_GUI");
}


//The Xserver must be started before this function (startxwin)
void* start_GUI ()
{
	GladeXML* xml;
	GtkWidget* main_window;
	GtkWidget* control_button;
	
	//Set DISPLAY enviroment variable if not set
	setenv ("DISPLAY", ":0", 0);

	//Initialize GTK+.
	if (!gtk_init_check(0,NULL))
	{
		fprintf (stderr, "Failed to initialize gtk\n");
		pthread_exit(NULL);
	}

	//Load the interface description.
	xml = glade_xml_new("FlightGear_comunicator_GUI.glade", NULL, NULL);
	if (!xml)
	{
		fprintf (stderr, "Failed to load glade xml\n");
		pthread_exit(NULL);
	}

	//glade_xml_signal_autoconnect(xml);
	
	//Find the main window (not shown by default, ogcalcmm.cc need sit to be hidden initially) and then show it.
	main_window = glade_xml_get_widget (xml,"FlightGear_comunicator_main_window");
	properties_text_conteiners[ROLL] = glade_xml_get_widget (xml,"Roll_value");
	properties_text_conteiners[PITCH] = glade_xml_get_widget (xml,"Pitch_value");
	properties_text_conteiners[HEADING] = glade_xml_get_widget (xml,"Heading_value");
	properties_text_conteiners[ROLL_RATE] = glade_xml_get_widget (xml,"Roll_rate_value");
	properties_text_conteiners[PITCH_RATE] = glade_xml_get_widget (xml,"Pitch_rate_value");
	properties_text_conteiners[YAW_RATE] = glade_xml_get_widget (xml,"Yaw_rate_value");
	properties_text_conteiners[AIRSPEED] = glade_xml_get_widget (xml,"Airspeed_value");
	properties_text_conteiners[U_BODY] = glade_xml_get_widget (xml,"U-body_value");
	properties_text_conteiners[V_BODY] = glade_xml_get_widget (xml,"V-body_value");
	properties_text_conteiners[W_BODY] = glade_xml_get_widget (xml,"W-body_value");
	properties_text_conteiners[NLF] = glade_xml_get_widget (xml,"NLF_value");
	properties_text_conteiners[X_ACCEL] = glade_xml_get_widget (xml,"X-accel_value");
	properties_text_conteiners[Y_ACCEL] = glade_xml_get_widget (xml,"Y-accel_value");
	properties_text_conteiners[Z_ACCEL] = glade_xml_get_widget (xml,"Z-accel_value");
	
	controls_text_conteiners[AILERON] = glade_xml_get_widget (xml,"Aileron_value");
	controls_text_conteiners[ELEVATOR] = glade_xml_get_widget (xml,"Elevator_value");
	controls_text_conteiners[RUDDER] = glade_xml_get_widget (xml,"Rudder_value");
	control_button = glade_xml_get_widget (xml, "Control_button");
	
	
	//Setup the signal handlers.
	glade_xml_signal_connect (xml, "on_window_destroy", G_CALLBACK (gtk_main_quit));
	g_signal_connect (G_OBJECT (control_button), "destroy", G_CALLBACK (stop_GUI), NULL);
	g_signal_connect (G_OBJECT (control_button), "clicked", G_CALLBACK (control_button_clicked), NULL);
	
	
	
	//Enter the GTK Event Loop. This is where all the events are caught and handled. It is exited with gtk main quit().
	gtk_widget_show (main_window);
	gtk_main();

	return 0;
}
