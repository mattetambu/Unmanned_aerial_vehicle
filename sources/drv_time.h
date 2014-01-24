// drv_time.h

#ifndef	_DRV_TIME_H_
#define	_DRV_TIME_H_

	#include <time.h>
	#include "common.h"
	
	/*
	 * Absolute time, in microsecond units.
	 *
	 * Absolute time is measured from some arbitrary epoch shortly after
	 * system startup.  It should never wrap or go backwards.
	 */
	typedef long int absolute_time;
	
	/* function prototypes */
	absolute_time get_absolute_time ();
	absolute_time get_absolute_time_since_start ();
	void timespec_to_absolute_time (struct timespec *ts, absolute_time *abs_time);
	void absolute_time_to_timespec (absolute_time abs_time, struct timespec *ts);
	absolute_time absolute_elapsed_time (const absolute_time *then);
	void system_time_init ();
	int set_timeout (absolute_time u_sec, void* function, void *arg);
		
	/* global variables */
	extern absolute_time UAV_start_time;

#endif
