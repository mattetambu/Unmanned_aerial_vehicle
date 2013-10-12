// plot.c

#include "common.h"
#include "plot.h"
#include "GUI.h"

int plot_flight_data (float* FDM_data)
{
	int return_value = 0, i;
	
	if (getenv("START_GUI"))
	{
		for (i = 0; i < N_USED_PROPERTIES; i++)
			if (properties_text_conteiners[i])
				gtk_label_set_text (GTK_LABEL (properties_text_conteiners[i]), g_strdup_printf ("%.6f", FDM_data[i]));
	}
	else
	{
		//Print data to screen for monitoring purposes
		return_value = fprintf (stdout, "\nFlight-data\n%f %f %f\n%f %f %f\n%f %f %f %f\n%f %f %f %f\n", 
					FDM_data[ROLL], FDM_data[PITCH], FDM_data[HEADING],
					FDM_data[ROLL_RATE], FDM_data[PITCH_RATE], FDM_data[YAW_RATE],
					FDM_data[AIRSPEED], FDM_data[U_BODY], FDM_data[V_BODY], FDM_data[W_BODY],
					FDM_data[NLF], FDM_data[X_ACCEL], FDM_data[Y_ACCEL], FDM_data[Z_ACCEL]);
		fflush (stdout);
	}
	
	return return_value;
}

int plot_flight_controls (float* controls)
{
	int return_value = 0, i;
	
	if (getenv("START_GUI"))
	{
		for (i = 0; i < N_USED_CONTROLS; i++)
			if (controls_text_conteiners[i])
				gtk_label_set_text (GTK_LABEL (controls_text_conteiners[i]), g_strdup_printf ("%.6f", controls[i]));
	}
	else
	{
		//Print data to screen for monitoring purposes
		return_value = fprintf (stdout, "\nFlight-controls\n%f %f %f\n", 
					controls[AILERON],
					controls[ELEVATOR],
					controls[RUDDER]);
		fflush (stdout);
	}
	
	return return_value;
}
