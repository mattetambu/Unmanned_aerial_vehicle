// main.h

#ifndef _MAIN_H
#define _MAIN_H
	
	/*
	fgfs --<option-list> --httpd=5500
		 --generic=socket,out,40,localhost,5000,tcp,output_protocol
		 --generic=socket,in,45,localhost,5100,tcp,input_protocol
		 --generic=socket,out,40,localhost,6000,udp,output_protocol
		 --generic=socket,in,45,localhost,6100,udp,input_protocol
	*/
	
	
	/* function prototypes */
	// empty
	
	/* global variables */
	extern float flight_data[N_USED_PROPERTIES];
	extern float flight_controls[N_USED_CONTROLS];

#endif
