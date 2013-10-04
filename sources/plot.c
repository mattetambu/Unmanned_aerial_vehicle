// plot.c

#include "common.h"
#include "plot.h"

int plot_flight_data (float* FDM_data)
{
	//Print data to screen for monitoring purposes
	return	fprintf (stdout, "\nFlight-data\n%f %f %f\n%f %f %f\n%f %f %f %f\n%f %f %f %f\n", 
				FDM_data[ROLL], FDM_data[PITCH], FDM_data[HEADING],
				FDM_data[ROLL_RATE], FDM_data[PITCH_RATE], FDM_data[YAW_RATE],
				FDM_data[AIRSPEED], FDM_data[U_BODY], FDM_data[V_BODY], FDM_data[W_BODY],
				FDM_data[NLF], FDM_data[X_ACCEL], FDM_data[Y_ACCEL], FDM_data[Z_ACCEL]);
}

int plot_flight_controls (float* controls)
{
	//Print data to screen for monitoring purposes
	return	fprintf (stdout, "\nFlight-controls\n%f %f %f\n", 
					controls[AILERON],
					controls[ELEVATOR],
					controls[RUDDER]);
}
