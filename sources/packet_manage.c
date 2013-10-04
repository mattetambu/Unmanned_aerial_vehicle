// packet_manage.c

#include "common.h"
#include "packet_manage.h"


int receive_input_packet (char* received_packet, int data_lenght, int input_socket)
{
	struct sockaddr_in address;
	int bytes_read, address_length = sizeof (struct sockaddr_in);
	
	if (getenv("TCP_FLAG"))
		bytes_read = recv (input_socket, received_packet, data_lenght, 0);
	else
	{ //Wait for new UDP packet, and read it into received_packet
		memset (&address, 0, address_length);
		bytes_read = recvfrom (input_socket, received_packet, data_lenght, 0, (struct sockaddr*) &address, (socklen_t*) &address_length);
	}
	
	if (bytes_read > 0)
	{
		received_packet[bytes_read] = '\0';
		if (getenv("VERBOSE"))
		{ //Print the raw data
			fprintf (stdout, "\nReceived packet:\n%s", received_packet);
			fflush (stdout);
		}
	}
	
	return bytes_read;
}

int parse_flight_data (float* FDM_data, char* received_packet)
{
	//Parse UDP data and store into float array
	return	sscanf (received_packet, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
					&FDM_data[ROLL], &FDM_data[PITCH], &FDM_data[HEADING],
					&FDM_data[ROLL_RATE], &FDM_data[PITCH_RATE], &FDM_data[YAW_RATE],
					&FDM_data[AIRSPEED], &FDM_data[U_BODY], &FDM_data[V_BODY], &FDM_data[W_BODY],
					&FDM_data[NLF], &FDM_data[X_ACCEL], &FDM_data[Y_ACCEL], &FDM_data[Z_ACCEL]);
}

int create_output_packet (char* output_packet, float* controls)
{
	//Construct a packet to send over UDP with flight controls	  
	return	sprintf (output_packet, "%f\t%f\t%f\n",
					controls[AILERON],
					controls[ELEVATOR],
					controls[RUDDER]);
}

int send_output_packet (char* output_packet, int data_lenght, int output_socket, struct sockaddr_in* address)
{
	int bytes_sent;
	
	if (getenv("TCP_FLAG"))
		bytes_sent = send (output_socket, output_packet, data_lenght, 0);
	else
		bytes_sent = sendto (output_socket, output_packet, data_lenght, 0, (struct sockaddr*) address, sizeof (struct sockaddr_in));
	
	if (bytes_sent == data_lenght && getenv("VERBOSE"))
	{ //Print Control inputs to stdout
		fprintf (stdout, "\nControl signals sent\n%s\n", output_packet);
		fflush (stdout);  		
	}
	
	return bytes_sent;
}
