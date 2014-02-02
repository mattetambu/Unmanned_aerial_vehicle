// comunicator_loop.c

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <signal.h>
#include <pthread.h>

#include "comunicator_loop.h"
#include "comunicator.h"
#include "socket_io.h"
#include "../common.h"
#include "../../simulator/FlightGear_exchanged_data.h"

#include "../../ORB/ORB.h"
#include "../../ORB/topics/airspeed.h"
#include "../../ORB/topics/vehicle_attitude.h"
#include "../../ORB/topics/vehicle_global_position.h"
#include "../../ORB/topics/actuator/actuator_controls.h"


pthread_t primary_thread_id;
int primary_thread_return_value = 0;

/* Advertise airspeed, vehicle_global_position, vehicle_attitude topic */
orb_advert_t airspeed_adv;
orb_advert_t vehicle_global_position_adv;
orb_advert_t vehicle_attitude_adv;
struct airspeed_s local_airspeed;
struct vehicle_global_position_s local_vehicle_global_position;
struct vehicle_attitude_s local_vehicle_attitude;
/* Subscribe to actuator_controls topic */
orb_subscr_t actuator_controls_sub;
struct actuator_controls_s local_actuator_controls;


int parse_flight_data (double* flight_data, char* received_packet)
{
	//Parse UDP data and store into double array
	return	sscanf (received_packet, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
					&flight_data[FDM_FLIGHT_TIME], &flight_data[FDM_TEMPERATURE], &flight_data[FDM_PRESSURE],
					&flight_data[FDM_LATITUDE], &flight_data[FDM_LONGITUDE],
					&flight_data[FDM_ALTITUDE], &flight_data[FDM_GROUND_LEVEL],
					&flight_data[FDM_ROLL_ANGLE], &flight_data[FDM_PITCH_ANGLE], &flight_data[FDM_YAW_ANGLE],
					&flight_data[FDM_ROLL_RATE], &flight_data[FDM_PITCH_RATE], &flight_data[FDM_YAW_RATE],
					&flight_data[FDM_X_BODY_VELOCITY], &flight_data[FDM_Y_BODY_VELOCITY], &flight_data[FDM_Z_BODY_VELOCITY],
					&flight_data[FDM_X_EARTH_VELOCITY], &flight_data[FDM_Y_EARTH_VELOCITY], &flight_data[FDM_Z_EARTH_VELOCITY], &flight_data[FDM_AIRSPEED],
					&flight_data[FDM_X_BODY_ACCELERATION], &flight_data[FDM_Y_BODY_ACCELERATION], &flight_data[FDM_Z_BODY_ACCELERATION],
					&flight_data[FDM_ENGINE_ROTATION_SPEED], &flight_data[FDM_ENGINE_THRUST], 
					&flight_data[FDM_MAGNETIC_VARIATION], &flight_data[FDM_MAGNETIC_DIP]);
}

int create_output_packet (char* output_packet)
{
	//Construct a packet to send over UDP with flight flight_controls	  
	return	sprintf (output_packet, "%lf\t%lf\t%lf\t%lf\n",
					local_actuator_controls.aileron,
					local_actuator_controls.elevator,
					local_actuator_controls.rudder,
					local_actuator_controls.throttle);
}

int publish_flight_data (double* flight_data)
{
	local_airspeed.indicated_airspeed_m_s = (float) flight_data[FDM_AIRSPEED];
	local_airspeed.true_airspeed_m_s = (float) flight_data[FDM_AIRSPEED];

	if (orb_publish (ORB_ID(airspeed), airspeed_adv, (const void *) &local_airspeed) < 0)
	{
		fprintf (stderr, "Failed to publish the airspeed topic\n");
		return -1;
	}
	
	local_vehicle_global_position.valid = 1;
	local_vehicle_global_position.yaw = (float) flight_data[FDM_YAW_ANGLE];
	local_vehicle_global_position.latitude = (double) flight_data[FDM_LATITUDE];
	local_vehicle_global_position.longitude = (double) flight_data[FDM_LONGITUDE];
	local_vehicle_global_position.altitude = (float) flight_data[FDM_ALTITUDE];
	local_vehicle_global_position.ground_level = (float) flight_data[FDM_GROUND_LEVEL];
	local_vehicle_global_position.vx = (float) flight_data[FDM_X_EARTH_VELOCITY];
	local_vehicle_global_position.vy = (float) flight_data[FDM_Y_EARTH_VELOCITY];
	local_vehicle_global_position.vz = (float) flight_data[FDM_Z_EARTH_VELOCITY];

	if (orb_publish (ORB_ID(vehicle_global_position), vehicle_global_position_adv, (const void *) &local_vehicle_global_position) < 0)
	{
		fprintf (stderr, "Failed to publish the airspeed topic\n");	
		return -1;
	}
	
	local_vehicle_attitude.roll =  (float) flight_data[FDM_ROLL_ANGLE];
	local_vehicle_attitude.pitch =  (float) flight_data[FDM_PITCH_ANGLE];
	local_vehicle_attitude.yaw =  (float) flight_data[FDM_YAW_ANGLE];
	local_vehicle_attitude.roll_rate =  (float) flight_data[FDM_ROLL_RATE];
	local_vehicle_attitude.pitch_rate =  (float) flight_data[FDM_PITCH_RATE];
	local_vehicle_attitude.yaw_rate =  (float) flight_data[FDM_YAW_RATE];
	local_vehicle_attitude.vx =  (float) flight_data[FDM_X_BODY_VELOCITY];
	local_vehicle_attitude.vy =  (float) flight_data[FDM_Y_BODY_VELOCITY];
	local_vehicle_attitude.vz =  (float) flight_data[FDM_Z_BODY_VELOCITY];
	local_vehicle_attitude.ax =  (float) flight_data[FDM_X_BODY_ACCELERATION];
	local_vehicle_attitude.ay =  (float) flight_data[FDM_Y_BODY_ACCELERATION];
	local_vehicle_attitude.az =  (float) flight_data[FDM_Z_BODY_ACCELERATION];
	local_vehicle_attitude.engine_rotation_speed =  (float) flight_data[FDM_ENGINE_ROTATION_SPEED];
	local_vehicle_attitude.thrust =  (float) flight_data[FDM_ENGINE_THRUST];
	
	if (orb_publish (ORB_ID(vehicle_attitude), vehicle_attitude_adv, (const void *) &local_vehicle_attitude) < 0)
	{
		fprintf (stderr, "Failed to publish the vehicle_attitude topic\n");	
		return -1;
	}
	
	return 0;
}


void* comunicator_loop (void* args)
{
	char received_data[INPUT_DATA_MAX_LENGTH];
	char data_sent[OUTPUT_DATA_MAX_LENGTH];
	double FlightGear_flight_data [FDM_N_PROPERTIES];		// FDM data
	struct sockaddr_in output_sockaddr;
	int wait_return_value;
	
	
	// ************************************* subscribe and advertise ******************************************
	airspeed_adv = orb_advertise (ORB_ID(airspeed));
	if (airspeed_adv == -1)
	{
		fprintf (stderr, "Primary thread can't advertise the airspeed topic\n");
		exit (-1);
	}

	vehicle_global_position_adv = orb_advertise (ORB_ID(vehicle_global_position));
	if (vehicle_global_position_adv == -1)
	{
		fprintf (stderr, "Primary thread can't advertise the vehicle_global_position topic\n");
		exit (-1);
	}

	vehicle_attitude_adv = orb_advertise (ORB_ID(vehicle_attitude));
	if (vehicle_attitude_adv == -1)
	{
		fprintf (stderr, "Primary thread can't advertise the vehicle_attitude topic\n");
		exit (-1);
	}

	actuator_controls_sub = orb_subscribe (ORB_ID(actuator_controls));
	if (actuator_controls_sub == -1)
	{
		fprintf (stderr, "Primary thread can't subscribe the vehicle_attitude topic\n");
		exit (-1);
	}

	
	// **************************************** init sockets ******************************************
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
			exit (-1);
		}
		if (!getenv("DO_NOT_SEND_CONTROLS") && tcp_client_socket_initialization (&output_socket, &output_sockaddr) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly\n");
			exit (-1);
		}
	}
	else
	{ //UDP
		if (udp_server_socket_initialization (&input_socket, input_port) < 0)
		{
			fprintf (stderr, "Can't create the input socket correctly\n");
			exit (-1);
		}
		
		if (!getenv("DO_NOT_SEND_CONTROLS") && udp_client_socket_initialization (&output_socket, output_port) < 0)
		{
			fprintf (stderr, "Can't create the output socket correctly\n");
			exit (-1);
		}

		fprintf (stdout, "Waiting for FlightGear on port %d\n\n", input_port);
		fflush(stdout);
	}

	
	// **************************************** start primary loop ******************************************
	while (!_shutdown_all_systems)
	{
		// INPUTS
		if (receive_input_packet (received_data, INPUT_DATA_MAX_LENGTH, input_socket) <= 0)
		{
			sleep(0.5);
			
			if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Can't receive the flight-data from FlightGear\n");
				primary_thread_return_value = -1;
			}
			
			break;
		}
		
		if (parse_flight_data (FlightGear_flight_data, received_data) < FDM_N_PROPERTIES)
		{
			//if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Can't parse the flight-data\n");
				primary_thread_return_value = -1;
			}
			
			break;
		}
		
		// ******************************************* publish *******************************************
		if (publish_flight_data (FlightGear_flight_data) < 0)
		{
			//if (!_shutdown_all_systems)
			{
				fprintf (stderr, "Can't publish flight-data\n");
				primary_thread_return_value = -1;
			}
			
			break;
		}
		

		if (!getenv("DO_NOT_SEND_CONTROLS"))
		{
			// wait for the result of the autopilot logic calculation
			wait_return_value = orb_wait (ORB_ID(actuator_controls), actuator_controls_sub);
			if (wait_return_value < 0)
			{
				if (!_shutdown_all_systems)
				{
					fprintf (stderr, "Primary thread experienced an error waiting for actuator_controls topic\n");
					primary_thread_return_value = -1;
				}
				
				break;
			}
			else if (wait_return_value)
				orb_copy (ORB_ID(actuator_controls), actuator_controls_sub, (void *) &local_actuator_controls);
			
			
			// OUTPUTS
			if (create_output_packet (data_sent) < CTRL_N_CONTROLS)
			{
				//if (!_shutdown_all_systems)
				{
					fprintf (stderr, "Can't create the packet to be sent to FlightGear\n");
					primary_thread_return_value = -1;
				}
				
				break;
			}
			
			if (send_output_packet (data_sent, strlen(data_sent), output_socket, &output_sockaddr) < strlen(data_sent))
			{
				sleep(0.5);
				
				if (!_shutdown_all_systems)
				{
					fprintf (stderr, "Can't send the packet to FlightGear\n");
					primary_thread_return_value = -1;
				}
				
				break;
			}
		}
	}
	
	
	// *********************************** unsubscribe and unadvertise ****************************************
	if (orb_unadvertise (ORB_ID(airspeed), airspeed_adv, pthread_self()) < 0)
		fprintf (stderr, "Failed to unadvertise the airspeed topic\n");

	if (orb_unadvertise (ORB_ID(vehicle_global_position), vehicle_global_position_adv, pthread_self()) < 0)
		fprintf (stderr, "Failed to unadvertise the vehicle_global_position topic\n");

	if (orb_unadvertise (ORB_ID(vehicle_attitude), vehicle_attitude_adv, pthread_self()) < 0)
		fprintf (stderr, "Failed to unadvertise the vehicle_attitude topic\n");

	if (orb_unsubscribe (ORB_ID(actuator_controls), actuator_controls_sub, pthread_self()) < 0)
		fprintf (stderr, "Failed to unsubscribe to actuator_controls topic\n");

	
	pthread_exit(&primary_thread_return_value);
	return 0;
}
