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


GtkWidget *main_window;
GtkWidget *properties_text_conteiners[FDM_N_PROPERTIES];
GtkWidget *controls_text_conteiners[CTRL_N_CONTROLS];

// flight data and mission textview updater
guint flight_data_updater_id;
guint mission_textview_updater_id;
pthread_t mission_textview_updater_pid = 0;
pthread_t flight_data_updater_pid = 0;

// topic subscriptions
orb_subscr_t GUI_mission_sub = -1;	/* Subscription to mission topic */
orb_subscr_t GUI_mission_small_sub = -1;	/* Subscription to mission_small topic */
orb_subscr_t GUI_airspeed_sub = -1;	/* Subscription to airspeed topic */
orb_subscr_t GUI_vehicle_global_position_sub = -1;	/* Subscription to vehicle_global_position topic */
orb_subscr_t GUI_vehicle_attitude_sub = -1;	/* Subscription to vehicle_attitude topic */
orb_subscr_t GUI_actuator_controls_sub = -1;	/* Subscription to actuator_controls topic */



int buffer_print_mission_command (mission_command_t *command, char *buffer_ptr, int text_length)
{
	int i, j, return_value;

	// should not happen
	if (!command)
		return -1;

	memset(buffer_ptr, '\0', text_length);

	for (i = 0; i < ((int) command->depth*N_SPACES_PER_TAB) && i < text_length;)
		for (j = 0; j < N_SPACES_PER_TAB; j++)
			buffer_ptr[i++] = ' ';


	if (command->name == accepted_command_end_repeat ||
		command->name == accepted_command_end_while ||
		command->name == accepted_command_end_if)
		{
			return_value = snprintf (buffer_ptr+i, text_length-i, "  }\n");
			return (return_value < 0)? -1 : i + return_value;
		}

	return_value = (command->name == accepted_command_repeat)?
					snprintf (buffer_ptr+i, text_length-i, "  while  ") :
					snprintf (buffer_ptr+i, text_length-i, "  %s  ", accepted_command_to_string(command->name));
	i += return_value;
	if (return_value < 0)
		return -1;

	if (command->id > 0)
	{
		return_value = snprintf (buffer_ptr+i, text_length-i, "id=%d  ", command->id);
		i += return_value;
		if (return_value < 0)
			return -1;
	}

	switch (command->name)
	{
		case accepted_command_rtl:
		case accepted_command_rth:
		case accepted_command_takeoff:
		case accepted_command_land:
			if (command->option1 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.3f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			break;
		case accepted_command_waypoint:
			if (command->option1 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.3f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}

			return_value = snprintf (buffer_ptr+i, text_length-i, "latitude=%.8f  longitude=%.8f  ", command->option2, command->option3);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;

		case accepted_command_loiter:
			if (command->option1 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.3f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			
			return_value = snprintf (buffer_ptr+i, text_length-i, "mode=%s  ", loiter_mode_to_string((loiter_mode_t) command->option2));
			i += return_value;
			if (return_value < 0)
				return -1;
			
			if (command->option3 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "seconds=%.1f  ", command->option3);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			else if (command->option3 < 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "rounds=%.0f  ", -command->option3);
				i += return_value;
				if (return_value < 0)
					return -1;
			}

			return_value = snprintf (buffer_ptr+i, text_length-i, "radius=%.1f  ", command->option4);
			i += return_value;
			if (return_value < 0)
				return -1;
			
			break;

		case accepted_command_delay:
			return_value = snprintf (buffer_ptr+i, text_length-i, "seconds=%.1f  ", command->option1);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;

		case accepted_command_jump:
			return_value = snprintf (buffer_ptr+i, text_length-i, "target_id=%.0f  ", command->option1);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;

		case accepted_command_set:
			return_value = snprintf (buffer_ptr+i, text_length-i, "variable=%s  value=%.3f  mode=%s  ",
									set_variable_to_string((set_variable_t) command->option1),
									command->option3,
									set_mode_to_string((set_mode_t) command->option2));
			i += return_value;
			if (return_value < 0)
				return -1;
			break;

		case accepted_command_repeat:
			return_value = snprintf (buffer_ptr+i, text_length-i, "(counter-%.0f-  <  %.0f)  {  ",
									command->option1,
									command->option3);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;

		case accepted_command_while:
		case accepted_command_if:
			return_value = snprintf (buffer_ptr+i, text_length-i, "(%s  %s  %.3f)  {  ",
									test_variable_to_string ((test_variable_t) command->option1),
									condition_sign_to_simbol ((condition_sign_t) command->option2),
									command->option3);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;

		default:
			// should not happen
			fprintf (stderr, "Invalid property \'name\' in a command in the mission command list\n");
			return -1;

	}
	return_value = snprintf (buffer_ptr+i, text_length-i, "\n");

	return (return_value < 0)? -1 : i + return_value;
}

void textview_buffer_fill_with_string (GtkTextBuffer *buffer, gchar *text_row, int text_length)
{
	GtkTextMark *mark;
	GtkTextIter iter;

	// GUI is closing
	if (!getenv("START_GUI") || getenv("GUI_STOPPED"))
		return;

	mark = gtk_text_buffer_get_insert (buffer);
	gtk_text_buffer_get_iter_at_mark (buffer, &iter, mark);
	gtk_text_buffer_insert (buffer, &iter, text_row, text_length);
}

int mission_textview_fill (GtkTextView * mission_textview, struct mission_s *local_mission) {
	gchar text_row [MAX_TEXT_ROW_LENGTH];
	int text_length;
	GtkTextBuffer *buffer;
	mission_command_index_t index;

	// GUI is closing
	if (!getenv("START_GUI") || getenv("GUI_STOPPED"))
		return 0;

	// safety check
	// should not happen
	if (!mission_textview)
	{
		return -1;
	}

	// get GTK thread lock
	gdk_threads_enter ();

	buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (mission_textview));

	text_length = snprintf ((char *) text_row, MAX_TEXT_ROW_LENGTH, "* mission  mode=%s  lastly=%s \n",
						mission_mode_to_string(local_mission->mode),
						mission_lastly_cmd_to_string(local_mission->lastly));
	textview_buffer_fill_with_string (buffer, text_row, text_length);

	for (index = 0; index < local_mission->n_commands; index++)
	{
		text_length = buffer_print_mission_command (&local_mission->command_list[index], (char*) text_row, MAX_TEXT_ROW_LENGTH);
		if (text_length < 0)
		{
			fprintf (stderr , "Failed to plot a mission command\n");
			// release GTK thread lock
			gdk_threads_leave ();
			return -1;
		}
		textview_buffer_fill_with_string (buffer, text_row, text_length);
	}

	text_length = snprintf ((char *) text_row, MAX_TEXT_ROW_LENGTH, "  mission end");
	textview_buffer_fill_with_string (buffer, text_row, text_length);

	// release GTK thread lock
	gdk_threads_leave ();

	return 0;
}

int init_mission_textview_updater (gpointer user_data)
{
	GtkTextView * mission_textview = (GtkTextView *) user_data;
	struct mission_s local_mission;
	int mission_wrv;
	
	// GUI is closing
	if (!getenv("START_GUI") || getenv("GUI_STOPPED"))
		return 0;

	if (GUI_mission_sub == -1)
	{
		// ********************** Subscribe to mission topics ******************************
		GUI_mission_sub = orb_subscribe (ORB_ID(mission));
		if (GUI_mission_sub < 0)
		{
			fprintf (stderr, "Failed to subscribe to mission topic\n");
			return -1;
		}

		mission_wrv = orb_wait (ORB_ID(mission), GUI_mission_sub);
		if (mission_wrv < 0 /* && !getenv ("GUI_STOPPED") */)
		{
			fprintf (stderr, "GUI thread experienced an error waiting for mission topic\n");
			return -1;
		}
		else if (mission_wrv)
			orb_copy (ORB_ID(mission), GUI_mission_sub, (void *) &local_mission);
		
		if (mission_textview_fill ((GtkTextView *) mission_textview, &local_mission) < 0)
		{
			fprintf (stderr, "Failed to fill the GUI mission textbox\n");
			return -1;
		}
	}
	
	if (GUI_mission_small_sub == -1)
	{
		// ********************** Subscribe to mission_small topics ******************************
		GUI_mission_small_sub = orb_subscribe (ORB_ID(mission_small));
		if (GUI_mission_small_sub < 0)
		{
			fprintf (stderr, "Failed to subscribe to mission topic\n");
			return -1;
		}
	}

	mission_textview_updater_pid = pthread_self();
	
	return 0;
}

int GUI_mission_textview_update (gpointer user_data)
{
	GtkTextView * mission_textview = (GtkTextView *) user_data;
	
	// orb data (prv means poll_return_value)
	int mission_small_prv;
	absolute_time usec_max_poll_wait_time = 250000;
	// topic data
	struct mission_small_s local_mission_small;

	if (!getenv("START_GUI") || getenv("GUI_STOPPED"))
		return 0;
	
	// safety check
	// should not happen
	if (!mission_textview)
	{
		fprintf (stderr, "Failed to get the GUI mission textbox\n");
		return 0;
	}

	if (GUI_mission_sub == -1 || GUI_mission_small_sub == -1)
	{
		if (init_mission_textview_updater (user_data) < 0)
		{
			fprintf (stdout, "Failed to initialize the mission textview\n");
			return 0;
		}
	}
	
	mission_small_prv = orb_poll (ORB_ID(mission_small), GUI_mission_small_sub, usec_max_poll_wait_time);
	if (mission_small_prv < 0 /* && !getenv ("GUI_STOPPED") */)
	{
		fprintf (stderr, "GUI thread experienced an error waiting for mission_small topic\n");
		return 0;
	}
	else if (mission_small_prv)
	{
		orb_copy (ORB_ID(mission_small), GUI_mission_small_sub, (void *) &local_mission_small);
		//fprintf (stdout, "GUI thread successfully copy the mission_small after poll\n");
		//fflush (stdout);
		
		// get GTK thread lock
		gdk_threads_enter ();

		// do the update - xxx not yet implemented
		//fprintf (stdout, "GUI thread successfully copy the mission for mission textview update\n");
		//fflush (stdout);

		// release GTK thread lock
		gdk_threads_leave ();
	}


	return (!getenv("START_GUI") || getenv("GUI_STOPPED"))? 0 : 1;
}

int init_flight_data_updater ()
{
	// GUI is closing
	if (!getenv("START_GUI") || getenv("GUI_STOPPED"))
		return 0;

	if (GUI_airspeed_sub == -1)
	{
		// ********************** Subscribe to airspeed topic ******************************
		GUI_airspeed_sub = orb_subscribe (ORB_ID(airspeed));
		if (GUI_airspeed_sub < 0)
		{
			fprintf (stderr, "Failed to subscribe to airspeed topic\n");
			return -1;
		}
	}
	
	if (GUI_vehicle_global_position_sub == -1)
	{
		// ********************** Subscribe to vehicle_global_position topic ******************************
		GUI_vehicle_global_position_sub = orb_subscribe (ORB_ID(vehicle_global_position));
		if (GUI_vehicle_global_position_sub < 0)
		{
			fprintf (stderr, "Failed to subscribe to vehicle_global_position topic\n");
			return -1;
		}
	}
	
	if (GUI_vehicle_attitude_sub == -1)
	{
		// ********************** Subscribe to vehicle_attitude topic ******************************
		GUI_vehicle_attitude_sub = orb_subscribe (ORB_ID(vehicle_attitude));
		if (GUI_vehicle_attitude_sub < 0)
		{
			fprintf (stderr, "Failed to subscribe to vehicle_attitude topic\n");
			return -1;
		}
	}
	
	if (GUI_actuator_controls_sub == -1)
	{
		// ********************** Subscribe to actuator_controls topic ******************************
		GUI_actuator_controls_sub = orb_subscribe (ORB_ID(actuator_controls));
		if (GUI_actuator_controls_sub < 0)
		{
			fprintf (stderr, "Failed to subscribe to actuator_controls topic\n");
			return -1;
		}
	}
	
	flight_data_updater_pid = pthread_self();
	
	return 0;
}

int GUI_flight_data_update ()
{
	int i;
	
	// orb data (prv means poll_return_value)
	int airspeed_prv, vehicle_global_position_prv, vehicle_attitude_prv, actuator_controls_prv = 0;
	absolute_time usec_max_poll_wait_time = 100000;
	
	// topics data
	struct airspeed_s local_airspeed;
	struct vehicle_global_position_s local_vehicle_global_position;
	struct vehicle_attitude_s local_vehicle_attitude;
	struct actuator_controls_s local_actuator_controls;
	
	if (!getenv("START_GUI") || getenv("GUI_STOPPED"))
		return 0;

	if (GUI_airspeed_sub == -1 || GUI_vehicle_global_position_sub == -1 || GUI_vehicle_attitude_sub == -1 || GUI_actuator_controls_sub == -1)
	{
		if (init_flight_data_updater () < 0)
		{
			fprintf (stdout, "Failed to initialize the GUI for flight data export\n");
			return 0;
		}
	}
	
	airspeed_prv = orb_poll (ORB_ID(airspeed), GUI_airspeed_sub, usec_max_poll_wait_time);
	if (airspeed_prv < 0 /* && !getenv ("GUI_STOPPED") */)
	{
		fprintf (stderr, "GUI thread experienced an error waiting for airspeed topic\n");
		return 0;
	}
	else if (airspeed_prv)
		orb_copy (ORB_ID(airspeed), GUI_airspeed_sub, (void *) &local_airspeed);
	
	vehicle_global_position_prv = orb_poll (ORB_ID(vehicle_global_position), GUI_vehicle_global_position_sub, usec_max_poll_wait_time);
	if (vehicle_global_position_prv < 0 /* && !getenv ("GUI_STOPPED") */)
	{
		fprintf (stderr, "GUI thread experienced an error waiting for vehicle_global_position topic\n");
		return 0;
	}
	else if (vehicle_global_position_prv)
		orb_copy (ORB_ID(vehicle_global_position), GUI_vehicle_global_position_sub, (void *) &local_vehicle_global_position);
	
	vehicle_attitude_prv = orb_poll (ORB_ID(vehicle_attitude), GUI_vehicle_attitude_sub, usec_max_poll_wait_time);
	if (vehicle_attitude_prv < 0 /* && !getenv ("GUI_STOPPED") */)
	{
		fprintf (stderr, "GUI thread experienced an error waiting for vehicle_attitude topic\n");
		return 0;
	}
	else if (vehicle_attitude_prv)
		orb_copy (ORB_ID(vehicle_attitude), GUI_vehicle_attitude_sub, (void *) &local_vehicle_attitude);
	
	actuator_controls_prv = orb_poll (ORB_ID(actuator_controls), GUI_actuator_controls_sub, usec_max_poll_wait_time);
	if (actuator_controls_prv < 0 /* && !getenv ("GUI_STOPPED") */)
	{
		fprintf (stderr, "GUI thread experienced an error waiting for actuator_controls topic\n");
		return 0;
	}
	else if (actuator_controls_prv)
		orb_copy (ORB_ID(actuator_controls), GUI_actuator_controls_sub, (void *) &local_actuator_controls);


	// get GTK thread lock
	gdk_threads_enter ();
	
#ifdef GTK_MULTITHREAD
	// lock the properties_text_conteiners variable
	G_LOCK_EXTERN (properties_text_conteiners);
#endif
	
	// safety check
	// should not happen
	for (i = FDM_LATITUDE; i <= FDM_ENGINE_THRUST; i++)
		if (!properties_text_conteiners[i])
		{
			fprintf (stderr, "Failed to get the GUI property text container\n");
			return 0;
		}

	if (vehicle_attitude_prv /* && !getenv ("GUI_STOPPED") */)
	{
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_ROLL_ANGLE]), g_strdup_printf ("%.6f", local_vehicle_attitude.roll));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_PITCH_ANGLE]), g_strdup_printf ("%.6f", local_vehicle_attitude.pitch));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_YAW_ANGLE]), g_strdup_printf ("%.6f", local_vehicle_attitude.yaw));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_ROLL_RATE]), g_strdup_printf ("%.6f", local_vehicle_attitude.roll_rate));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_PITCH_RATE]), g_strdup_printf ("%.6f", local_vehicle_attitude.pitch_rate));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_YAW_RATE]), g_strdup_printf ("%.6f", local_vehicle_attitude.yaw_rate));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_X_BODY_VELOCITY]), g_strdup_printf ("%.6f", local_vehicle_attitude.vx));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_Y_BODY_VELOCITY]), g_strdup_printf ("%.6f", local_vehicle_attitude.vy));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_Z_BODY_VELOCITY]), g_strdup_printf ("%.6f", local_vehicle_attitude.vz));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_X_BODY_ACCELERATION]), g_strdup_printf ("%.6f", local_vehicle_attitude.ax));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_Y_BODY_ACCELERATION]), g_strdup_printf ("%.6f", local_vehicle_attitude.ay));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_Z_BODY_ACCELERATION]), g_strdup_printf ("%.6f", local_vehicle_attitude.az));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_ENGINE_ROTATION_SPEED]), g_strdup_printf ("%.6f", local_vehicle_attitude.engine_rotation_speed));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_ENGINE_THRUST]), g_strdup_printf ("%.6f", local_vehicle_attitude.thrust));
	}
	if (airspeed_prv /* && !getenv ("GUI_STOPPED") */)
	{
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_AIRSPEED]), g_strdup_printf ("%.6f", local_airspeed.indicated_airspeed_m_s));
	}
	if (vehicle_global_position_prv /* && !getenv ("GUI_STOPPED") */)
	{
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_LATITUDE]), g_strdup_printf ("%.10f", local_vehicle_global_position.latitude));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_LONGITUDE]), g_strdup_printf ("%.10f", local_vehicle_global_position.longitude));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_ALTITUDE]), g_strdup_printf ("%.6f", local_vehicle_global_position.altitude));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_GROUND_LEVEL]), g_strdup_printf ("%.6f", local_vehicle_global_position.ground_level));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_X_EARTH_VELOCITY]), g_strdup_printf ("%.6f", local_vehicle_global_position.vx));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_Y_EARTH_VELOCITY]), g_strdup_printf ("%.6f", local_vehicle_global_position.vy));
		gtk_label_set_text (GTK_LABEL (properties_text_conteiners[FDM_Z_EARTH_VELOCITY]), g_strdup_printf ("%.6f", local_vehicle_global_position.vz));
	}

#ifdef GTK_MULTITHREAD			
	// unlock the properties_text_conteiners variable
	G_UNLOCK (properties_text_conteiners);
	
	// lock the controls_text_conteiners variable
	G_LOCK_EXTERN (controls_text_conteiners);
#endif
	
	// safety check
	// should not happen
	for (i = 0; i < CTRL_N_CONTROLS; i++)
		if (!controls_text_conteiners[i])
		{
			fprintf (stderr, "Failed to get the GUI control text container\n");
			return 0;
		}
	
	if (actuator_controls_prv)
	{
		if  (!getenv("DO_NOT_SEND_CONTROLS") /* && !getenv ("GUI_STOPPED") */)
		{
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[CTRL_AILERON]), g_strdup_printf ("%.6f", local_actuator_controls.aileron));
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[CTRL_ELEVATOR]), g_strdup_printf ("%.6f", local_actuator_controls.elevator));
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[CTRL_RUDDER]), g_strdup_printf ("%.6f", local_actuator_controls.rudder));
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[CTRL_THROTTLE]), g_strdup_printf ("%.6f", local_actuator_controls.throttle));
		}
		else if (1 /* && !getenv ("GUI_STOPPED") */)
		{
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[CTRL_AILERON]), "-");
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[CTRL_ELEVATOR]), "-");
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[CTRL_RUDDER]), "-");
			gtk_label_set_text (GTK_LABEL (controls_text_conteiners[CTRL_THROTTLE]), "-");
		}
	}

#ifdef GTK_MULTITHREAD
	// unlock the controls_text_conteiners variable
	G_UNLOCK (controls_text_conteiners);
#endif

	// release GTK thread lock
	gdk_threads_leave ();
	

	return (!getenv("START_GUI") || getenv("GUI_STOPPED"))? 0 : 1;
}

