// GUI.c
#include "GUI.h"
#include "common.h"

//The Xserver must be started before this function (startxwin)
void* start_GUI ()
{
	GladeXML* xml;
	GtkWidget* window;

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

	//Setup the signal handlers.
	glade_xml_signal_autoconnect(xml);

	//Find the main window (not shown by default, ogcalcmm.cc need sit to be hidden initially) and then show it.
	window=glade_xml_get_widget(xml,"FlightGear_comunicator_main_window");

	gtk_widget_show(window);
	//Enter the GTK Event Loop. This is where all the events are caught and handled. It is exited with gtk main quit().
	gtk_main();

	return 0;
}

/*#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gtk/gtk.h>
#include <gdk/gdkx.h>
#include <glade/glade.h> // NOTE: Include this for libglade

GladeXML *xml;

// Create MyWindow
if (!(xml = glade_xml_new("Myfile.glade", "MyWindow", 0)))
printf("MyWindow failed to load\n");
else
{
GtkWidget *newWidget;

// Get the widget with an id of "MyButton"
newWidget = glade_xml_get_widget(xml, "MyButton");

// Attach my on_click() function to its "clicked"" event
g_signal_connect(G_OBJECT(newWidget), "clicked", G_CALLBACK(on_clicked), xml);
}*/
