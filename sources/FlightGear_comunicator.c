// FlightGear_comunicator.c

#include <signal.h>
#include <pthread.h>
#include "common.h"
#include "socket_io.h"
#include "FlightGear_comunicator.h"
#include "autopilot_logic.h"
#include "autopilot_parameters.h"
#include "plot.h"

int input_port = -1, output_port = -1;
char output_address[20];

int comunicator_thread_return_value = 0;
int SHUTDOWN_COMUNICATOR = 0;
pthread_mutex_t mutex;


int parse_flight_data (double* flight_data, char* received_packet)
{
	//Parse UDP data and store into double array
	return	sscanf (received_packet, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
					&flight_data[FLIGHT_TIME], &flight_data[TEMPERATURE], &flight_data[PRESSURE],
					&flight_data[LATITUDE], &flight_data[LONGITUDE],
					&flight_data[ALTITIUDE], &flight_data[GROUND_LEVEL],
					&flight_data[ROLL_ANGLE], &flight_data[PITCH_ANGLE], &flight_data[YAW_ANGLE],
					&flight_data[ROLL_RATE], &flight_data[PITCH_RATE], &flight_data[YAW_RATE],
					&flight_data[U_VELOCITY], &flight_data[V_VELOCITY], &flight_data[W_VELOCITY],
					&flight_data[NORTH_VELOCITY], &flight_data[EAST_VELOCITY], &flight_data[DOWN_VELOCITY], &flight_data[AIRSPEED],
					&flight_data[X_ACCELERATION], &flight_data[Y_ACCELERATION], &flight_data[Z_ACCELERATION],
					&flight_data[NORTH_ACCELERATION], &flight_data[EAST_ACCELERATION], &flight_data[DOWN_ACCELERATION],
					&flight_data[ENGINE_RPM],
					&flight_data[MAGNETIC_VARIATION], &flight_data[MAGNETIC_DIP]);
}

int create_output_packet (char* output_packet, double* flight_controls)
{
	//Construct a packet to send over UDP with flight flight_controls	  
	return	sprintf (output_packet, "%f\t%f\t%f\t%f\n",
					flight_controls[AILERON],
					flight_controls[ELEVATOR],
					flight_controls[RUDDER],
					flight_controls[THROTTLE]);
}


void* FlightGear_comunicator (void* args)
{
	char received_data[INPUT_DATA_MAX_LENGTH];
	char data_sent[OUTPUT_DATA_MAX_LENGTH];
	struct sockaddr_in output_sockaddr;
		
	if (!getenv("DO_NOT_SEND_CONTROLS") && client_address_initialization (&output_sockaddr, output_port, output_address) < 0)
	{
		fprintf (stderr, "Can't find the host on the given IP\n");
		comunicator_thread_return_value = -1;
		pthread_exit(&comunicator_thread_return_value);
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
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
		if (!getenv("DO_NOT_SEND_CONTROLS") && tcp_client_socket_initialization (&output_socket, &output_sockaddr) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
	}
	else
	{ //UDP
		if (udp_server_socket_initialization (&input_socket, input_port) < 0)
		{
			fprintf (stderr, "Can't create the input socket correctly\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
		
		if (!getenv("DO_NOT_SEND_CONTROLS") && udp_client_socket_initialization (&output_socket, output_port) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
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
			pthread_mutex_lock (&mutex);
			if (!SHUTDOWN_COMUNICATOR)
			{
				fprintf (stderr, "Can't receive the flight-data from FlightGear\n");
				comunicator_thread_return_value = -1;
			}
			pthread_mutex_unlock (&mutex);
			pthread_exit(&comunicator_thread_return_value);
		}
		
		if (parse_flight_data (aircraft->flight_data, received_data) < N_PROPERTIES)
		{
			fprintf (stderr, "Can't parse the flight-data\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
		
		if (plot_flight_data (aircraft->flight_data) < 0)
		{
			fprintf (stderr, "Can't plot the flight-data\n");
			comunicator_thread_return_value = -1;
			pthread_exit(&comunicator_thread_return_value);
		}
		
		if (!getenv("DO_NOT_SEND_CONTROLS"))
		{
			// AUTOPILOT LOGIC
			if (compute_flight_controls (aircraft->flight_controls, aircraft->flight_data) < 0)
			{
				fprintf (stderr, "Can't compute flight controls\n");
				comunicator_thread_return_value = -1;
				pthread_exit(&comunicator_thread_return_value);
			}
						
			// OUTPUTS
			if (plot_flight_controls (aircraft->flight_controls) < 0)
			{
				fprintf (stderr, "Can't plot the flight-controls\n");
				comunicator_thread_return_value = -1;
				pthread_exit(&comunicator_thread_return_value);
			}
			
			if (create_output_packet (data_sent, aircraft->flight_controls) < N_CONTROLS)
			{
				fprintf (stderr, "Can't create the packet to be sent to FlightGear\n");
				comunicator_thread_return_value = -1;
				pthread_exit(&comunicator_thread_return_value);
			}
			
			if (send_output_packet (data_sent, strlen(data_sent), output_socket, &output_sockaddr) < strlen(data_sent))
			{
				sleep(1);
				pthread_mutex_lock (&mutex);
				if (!SHUTDOWN_COMUNICATOR)
				{
					fprintf (stderr, "Can't send the packet to FlightGear\n");
					comunicator_thread_return_value = -1;
				}
				pthread_mutex_unlock (&mutex);
				pthread_exit(&comunicator_thread_return_value);
			}
		}
	}
	
	pthread_exit(&comunicator_thread_return_value);
	return 0;
}
