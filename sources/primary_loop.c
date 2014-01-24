// primary_loop.c

#include "primary_loop.h"
#include "common.h"
#include "comunicator.h"
#include "autopilot.h"
#include "aircraft.h"

pthread_t primary_thread_id;
int primary_thread_return_value = 0;


int parse_flight_data (double* flight_data, char* received_packet)
{
	//Parse UDP data and store into double array
	return	sscanf (received_packet, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
					&flight_data[FDM_FLIGHT_TIME], &flight_data[FDM_TEMPERATURE], &flight_data[FDM_PRESSURE],
					&flight_data[FDM_LATITUDE], &flight_data[FDM_LONGITUDE],
					&flight_data[FDM_ALTITIUDE], &flight_data[FDM_GROUND_LEVEL],
					&flight_data[FDM_ROLL_ANGLE], &flight_data[FDM_PITCH_ANGLE], &flight_data[FDM_YAW_ANGLE],
					&flight_data[FDM_ROLL_RATE], &flight_data[FDM_PITCH_RATE], &flight_data[FDM_YAW_RATE],
					&flight_data[FDM_U_VELOCITY], &flight_data[FDM_V_VELOCITY], &flight_data[FDM_W_VELOCITY],
					&flight_data[FDM_NORTH_VELOCITY], &flight_data[FDM_EAST_VELOCITY], &flight_data[FDM_DOWN_VELOCITY], &flight_data[FDM_AIRSPEED],
					&flight_data[FDM_X_ACCELERATION], &flight_data[FDM_Y_ACCELERATION], &flight_data[FDM_Z_ACCELERATION],
					&flight_data[FDM_NORTH_ACCELERATION], &flight_data[FDM_EAST_ACCELERATION], &flight_data[FDM_DOWN_ACCELERATION],
					&flight_data[FDM_ENGINE_RPM],
					&flight_data[FDM_MAGNETIC_VARIATION], &flight_data[FDM_MAGNETIC_DIP]);
}

int create_output_packet (char* output_packet, double* flight_controls)
{
	//Construct a packet to send over UDP with flight flight_controls	  
	return	sprintf (output_packet, "%lf\t%lf\t%lf\t%lf\n",
					flight_controls[CTRL_AILERON],
					flight_controls[CTRL_ELEVATOR],
					flight_controls[CTRL_RUDDER],
					flight_controls[CTRL_THROTTLE]);
}


void* primary_loop (void* args)
{
	char received_data[INPUT_DATA_MAX_LENGTH];
	char data_sent[OUTPUT_DATA_MAX_LENGTH];
	struct sockaddr_in output_sockaddr;
		
	if (!getenv("DO_NOT_SEND_CONTROLS") && client_address_initialization (&output_sockaddr, output_port, output_address) < 0)
	{
		fprintf (stderr, "Can't find the host on the given IP\n");
		primary_thread_return_value = -1;
		pthread_exit(&primary_thread_return_value);
	}
	
	fprintf (stdout, "\nTrying to establish a connection with FlightGear\n");
	fflush(stdout);
	
	if (getenv("TCP_FLAG"))
	{ //TCP
		fprintf (stdout, "Waiting for FlightGear on port %d\n\n", input_port);
		fflush(stdout);
		
		if (tcp_server_socket_initialization (&input_socket, input_port) < 0)
		{
			fprintf (stderr, "Can't create the input socket correctly\n");
			primary_thread_return_value = -1;
			pthread_exit(&primary_thread_return_value);
		}
		if (!getenv("DO_NOT_SEND_CONTROLS") && tcp_client_socket_initialization (&output_socket, &output_sockaddr) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly\n");
			primary_thread_return_value = -1;
			pthread_exit(&primary_thread_return_value);
		}
	}
	else
	{ //UDP
		if (udp_server_socket_initialization (&input_socket, input_port) < 0)
		{
			fprintf (stderr, "Can't create the input socket correctly\n");
			primary_thread_return_value = -1;
			pthread_exit(&primary_thread_return_value);
		}
		
		if (!getenv("DO_NOT_SEND_CONTROLS") && udp_client_socket_initialization (&output_socket, output_port) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly\n");
			primary_thread_return_value = -1;
			pthread_exit(&primary_thread_return_value);
		}

		fprintf (stdout, "Waiting for FlightGear on port %d\n\n", input_port);
		fflush(stdout);
	}
	
	
	while (1)
	{
		// INPUTS
		if (receive_input_packet (received_data, INPUT_DATA_MAX_LENGTH, input_socket) <= 0)
		{
			sleep(1);
			// pthread_mutex_lock (&shutdown_mutex);
			if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Can't receive the flight-data from FlightGear\n");
				primary_thread_return_value = -1;
			}
			// pthread_mutex_unlock (&shutdown_mutex);
			pthread_exit(&primary_thread_return_value);
		}
		
		if (parse_flight_data (aircraft->flight_data, received_data) < FDM_N_PROPERTIES)
		{
			fprintf (stderr, "Can't parse the flight-data\n");
			primary_thread_return_value = -1;
			pthread_exit(&primary_thread_return_value);
		}
		
		
		if (!getenv("DO_NOT_SEND_CONTROLS"))
		{
			// AUTOPILOT LOGIC
			// xxx - to be moved out
			if (compute_flight_controls (aircraft->flight_controls, aircraft->flight_data) < 0)
			{
				fprintf (stderr, "Can't compute flight controls\n");
				primary_thread_return_value = -1;
				pthread_exit(&primary_thread_return_value);
			}
			
			// OUTPUTS
			if (create_output_packet (data_sent, aircraft->flight_controls) < CTRL_N_CONTROLS)
			{
				fprintf (stderr, "Can't create the packet to be sent to FlightGear\n");
				primary_thread_return_value = -1;
				pthread_exit(&primary_thread_return_value);
			}
			
			if (send_output_packet (data_sent, strlen(data_sent), output_socket, &output_sockaddr) < strlen(data_sent))
			{
				sleep(1);
				// pthread_mutex_lock (&shutdown_mutex);
				if (!_shutdown_all_systems)
				{
					fprintf (stderr, "Can't send the packet to FlightGear\n");
					primary_thread_return_value = -1;
				}
				// pthread_mutex_unlock (&shutdown_mutex);
				pthread_exit(&primary_thread_return_value);
			}
		}
	}
	
	pthread_exit(&primary_thread_return_value);
	return 0;
}
