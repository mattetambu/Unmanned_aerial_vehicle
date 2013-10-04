// aautopilot_logic.c

#include "common.h"
#include "autopilot_logic.h"

int compute_flight_controls (float* controls, float* FDM_data)
{
	int i = 0;
	
	//Proportional control for roll and pitch
	controls[AILERON] = -0.05*(FDM_data[ROLL] - 0);
	controls[ELEVATOR] = 0.1*(FDM_data[PITCH] - 5);
	controls[RUDDER] = -0.05*(FDM_data[HEADING] - 0);

	//Limit control inputs
	for (i = 0; i < N_USED_CONTROLS; i++)
		if (fabs(controls[i]) > 0.6)
			controls[i] = (controls[i]/fabs(controls[i]))*0.6;
			
	return 0;
}
