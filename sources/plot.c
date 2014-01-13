// plot.c

#include "plot.h"


void plot_flight_data (double* flight_data)
{
	//Print data to screen for monitoring purposes
	fprintf (stdout, "\nFlight-data\n\
			Flight time: %.6f,\tTemperature: %.6f,\tPressure: %.6f\n\
			Latitude: %.10f,\tLongitude: %.10f\n\
			Altitude: %.6f,\tGround level: %.6f\n\
			Roll angle: %.6f,\tPitch angle: %.6f,\tYaw angle: %.6f\n\
			Roll rate: %.6f,\tPitch rate: %.6f,\tYaw rate: %.6f\n\
			U velocity: %.6f,\tV velocity: %.6f,\tW velocity: %.6f\n\
			North velocity: %.6f,\tEast velocity: %.6f,\tDown velocity: %.6f,\tAirspeed: %.6f\n\
			X acceleration: %.6f,\tY acceleration: %.6f,\tZ acceleration: %.6f\n\
			North acceleration: %.6f,\tEast acceleration: %.6f,\tDown acceleration: %.6f\n\
			Engine rotation speed: %.6f\n\
			Magnetic variation: %.6f,\tMagnetic dip: %.6f\n",
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

void plot_flight_controls (double* flight_controls)
{
	//Print data to screen for monitoring purposes
	fprintf (stdout, "\nFlight_controls\n\
			Aileron:\t%.6f\n\
			Elevator:\t%.6f\n\
			Rudder:\t\t%.6f\n\
			Throttle:\t%.6f\n",
			flight_controls[AILERON],
			flight_controls[ELEVATOR],
			flight_controls[RUDDER],
			flight_controls[THROTTLE]);

	fflush (stdout);
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
			
			return_value = snprintf (buffer_ptr+i, text_length-i, "latitude=%.8f  longitude=%.8f  ", command->option2, command->option3.dbl);
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
			if (command->option2 > 0)
			{
				return_value = snprintf (buffer_ptr+i, text_length-i, "seconds=%.1f  ", command->option2);
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
			return_value = snprintf (buffer_ptr+i, text_length-i, "seconds=%.1f  ", command->option2);
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
			return_value = snprintf (buffer_ptr+i, text_length-i, "variable=%s  value=%.3f  mode=%s  ",
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
			return_value = snprintf (buffer_ptr+i, text_length-i, "(%s  %s  %.3f)  {  ",
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

