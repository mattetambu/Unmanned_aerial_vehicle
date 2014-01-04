// plot.c

#include "common.h"
#include "autopilot_parameters.h"
#include "mission_logic.h"
#include "plot.h"
#include "GUI.h"

int plot_flight_data (double* flight_data)
{
	int return_value = 0, i;
	
	if (getenv("START_GUI"))
	{
		for (i = LATITUDE; i <= LONGITUDE; i++)
			if (properties_text_conteiners[i])
				gtk_label_set_text (GTK_LABEL (properties_text_conteiners[i]), g_strdup_printf ("%.18f", flight_data[i]));
		for (; i <= ENGINE_RPM; i++)
			if (properties_text_conteiners[i])
				gtk_label_set_text (GTK_LABEL (properties_text_conteiners[i]), g_strdup_printf ("%.6f", flight_data[i]));
	}
	else
	{
		//Print data to screen for monitoring purposes
		return_value = fprintf (stdout, "\nFlight-data\n\
			Flight time: %.8f,\tTemperature: %.8f,\tPressure: %.8f\n\
			Latitude: %.18f,\tLongitude: %.18f\n\
			Altitude: %.8f,\tGround level: %.8f\n\
			Roll angle: %.8f,\tPitch angle: %.8f,\tYaw angle: %.8f\n\
			Roll rate: %.8f,\tPitch rate: %.8f,\tYaw rate: %.8f\n\
			U velocity: %.8f,\tV velocity: %.8f,\tW velocity: %.8f\n\
			North velocity: %.8f,\tEast velocity: %.8f,\tDown velocity: %.8f,\tAirspeed: %.8f\n\
			X acceleration: %.8f,\tY acceleration: %.8f,\tZ acceleration: %.8f\n\
			North acceleration: %.8f,\tEast acceleration: %.8f,\tDown acceleration: %.8f\n\
			Engine rotation speed: %.8f\n\
			Magnetic variation: %.8f,\tMagnetic dip: %.8f\n",
			flight_data[FLIGHT_TIME], flight_data[TEMPERATURE], flight_data[PRESSURE],
			flight_data[LATITUDE], flight_data[LONGITUDE],
			flight_data[ALTITIUDE], flight_data[GROUND_LEVEL],
			flight_data[ROLL_ANGLE], flight_data[PITCH_ANGLE], flight_data[YAW_ANGLE],
			flight_data[ROLL_RATE], flight_data[PITCH_RATE], flight_data[YAW_RATE],
			flight_data[U_VELOCITY], flight_data[V_VELOCITY], flight_data[W_VELOCITY],
			flight_data[NORTH_VELOCITY], flight_data[EAST_VELOCITY], flight_data[DOWN_VELOCITY], flight_data[AIRSPEED],
			flight_data[X_ACCELERATION], flight_data[Y_ACCELERATION], flight_data[Z_ACCELERATION],
			flight_data[NORTH_ACCELERATION], flight_data[EAST_ACCELERATION], flight_data[DOWN_ACCELERATION],
			flight_data[ENGINE_RPM],
			flight_data[MAGNETIC_VARIATION], flight_data[MAGNETIC_DIP]);
			fflush (stdout);
	}
	
	return return_value;
}

int plot_flight_controls (double* flight_controls)
{
	int return_value = 0, i;
	
	if (getenv("START_GUI"))
	{
		for (i = 0; i < N_CONTROLS; i++)
			if (controls_text_conteiners[i])
				gtk_label_set_text (GTK_LABEL (controls_text_conteiners[i]), g_strdup_printf ("%.6f", flight_controls[i]));
	}
	else
	{
		//Print data to screen for monitoring purposes
		return_value = fprintf (stdout, "\nFlight_controls\n\
			Aileron:\t%.8f\n\
			Elevator:\t%.8f\n\
			Rudder:\t\t%.8f\n\
			Throttle:\t%.8f\n",
			flight_controls[AILERON],
			flight_controls[ELEVATOR],
			flight_controls[RUDDER],
			flight_controls[THROTTLE]);
		fflush (stdout);
	}
	
	return return_value;
}

int plot_mission_command (mission_command_t *command, char *buffer_ptr, int text_length)
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
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.1f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			break;
		case accepted_command_waypoint:
			if (command->option1 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.1f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			
			return_value = snprintf (buffer_ptr+i, text_length-i, "latitude=%.8f  longitude=%.8f  ", command->option2, command->option3.dbl);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_loiter:
			if (command->option1 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "altitude=%.1f  ", command->option1);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			if (command->option2 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "seconds=%.0f  ", command->option2);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			else if (command->option2 < 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "rounds=%.0f  ", -command->option2);
				i += return_value;
				if (return_value < 0)
					return -1;
			}
			
			return_value = snprintf (buffer_ptr+i, text_length-i, "mode=%s  ", loiter_mode_to_string((loiter_mode_t) command->option3.dbl));
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_delay:
			return_value = snprintf (buffer_ptr+i, text_length-i, "seconds=%.0f  ", command->option2);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_jump:
			return_value = snprintf (buffer_ptr+i, text_length-i, "target_id=%.0f  ", command->option2);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_set:
			return_value = snprintf (buffer_ptr+i, text_length-i, "variable=%s  value=%.1f  mode=%s  ",
									set_variable_to_string((set_variable_t) command->option1),
									command->option2,
									set_mode_to_string((set_mode_t) command->option3.dbl));
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
			
		case accepted_command_repeat:
			return_value = snprintf (buffer_ptr+i, text_length-i, "(counter  <  %.0f)  {  ", command->option2);
			i += return_value;
			if (return_value < 0)
				return -1;
			break;
		
		case accepted_command_while:
		case accepted_command_if:
			return_value = snprintf (buffer_ptr+i, text_length-i, "(%s  %s  %.1f)  {  ",
									test_variable_to_string ((test_variable_t) command->option2),
									condition_sign_to_simbol ((condition_sign_t) command->option3.cmd_ptr->option1),
									command->option3.cmd_ptr->option2);
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

