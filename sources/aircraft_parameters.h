// aircraft_parameters.h

#ifndef	_AIRCRAFT_PARAMETERS_H_
#define	_AIRCRAFT_PARAMETERS_H_

	#include "common.h"
	
	enum 
	{
		// FDM (AIRCRAFT SETTINGS)
		//	FLIGHT CONDITIONS
		FDM_FLIGHT_TIME = 0,
		FDM_TEMPERATURE,
		FDM_PRESSURE,
		//	POSITION
		FDM_LATITUDE,
		FDM_LONGITUDE,
		FDM_ALTITIUDE,
		FDM_GROUND_LEVEL,
		//	ORIENTATION
		FDM_ROLL_ANGLE,
		FDM_PITCH_ANGLE,
		FDM_YAW_ANGLE,
		FDM_ROLL_RATE,
		FDM_PITCH_RATE,
		FDM_YAW_RATE,
		//	VELOCITIES
		FDM_U_VELOCITY,
		FDM_V_VELOCITY,
		FDM_W_VELOCITY,
		FDM_NORTH_VELOCITY,
		FDM_EAST_VELOCITY,
		FDM_DOWN_VELOCITY,
		FDM_AIRSPEED,
		//	ACCELERATION
		FDM_X_ACCELERATION,
		FDM_Y_ACCELERATION,
		FDM_Z_ACCELERATION,
		FDM_NORTH_ACCELERATION,
		FDM_EAST_ACCELERATION,
		FDM_DOWN_ACCELERATION,
		//	ENGINE
		FDM_ENGINE_RPM,
		//  ENVIROMENT
		FDM_MAGNETIC_VARIATION,
		FDM_MAGNETIC_DIP,
		FDM_N_PROPERTIES
	};
	
	enum 
	{
		// CONTROLS
		//	FLIGHT CONTROLS
		CTRL_AILERON = 0,
		CTRL_ELEVATOR,
		CTRL_RUDDER,
		CTRL_THROTTLE,
		CTRL_N_CONTROLS
	};
	
	/* function prototypes */
	//empty
	
	/* global variables */
	//empty
	
#endif
