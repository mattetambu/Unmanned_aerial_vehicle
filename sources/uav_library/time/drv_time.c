// drv_time.c

#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <pthread.h>
#include "drv_time.h"
#include "../common.h"


struct timer_parameters
{
	absolute_time sleep_time;
	void * (*function_to_call) (void *);
	void *arg;
};

/*
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
absolute_time UAV_start_time;


/*
 * Return the absolute time
 */
absolute_time get_absolute_time ()
{
	struct timeval tv_time;
	absolute_time abs_time = 0;
	
	gettimeofday (&tv_time, NULL);
	abs_time = (absolute_time) (tv_time.tv_sec) * 1000000;
	abs_time += (absolute_time) tv_time.tv_usec;

	return abs_time;
}

/*
 * Return the relative time since system start
 */
absolute_time get_absolute_time_since_start ()
{
	absolute_time rel_time = absolute_elapsed_time (&UAV_start_time);

	return rel_time;
}

/*
 * Convert a timespec to absolute time
 */
void timespec_to_absolute_time (struct timespec *ts, absolute_time *abs_time)
{
	*abs_time = (absolute_time)(ts->tv_sec) * 1000000;
	*abs_time += ts->tv_nsec / 1000;
}

/*
 * Convert absolute time to a timespec.
 */
void absolute_time_to_timespec (absolute_time abs_time, struct timespec *ts)
{
	ts->tv_sec = abs_time / 1000000;
	abs_time -= ts->tv_sec * 1000000;
	ts->tv_nsec = abs_time * 1000;
}

/*
 * Compare a time value with the current time.
 */
absolute_time absolute_elapsed_time (const absolute_time *then)
{
	absolute_time delta;
	delta = get_absolute_time() - *then;

	return delta;
}

void system_time_init ()
{
	absolute_time _now = get_absolute_time ();
	UAV_start_time = _now;
}


// timer function
void *timer (void *data)
{
	struct timer_parameters *tp = (struct timer_parameters *) data;
	
	usleep (tp->sleep_time);
	tp->function_to_call (tp->arg);

	return NULL;
}

int set_timeout (absolute_time usec, void* function, void *arg)
{
	pthread_t thread_id;
	struct timer_parameters *params;
	
	if (usec < 0)
	{
		// nothing will be done
#ifdef DEBUG
		fprintf (stderr, "Trying to set a timeout whit negative sleep time\n");
#endif
		return 0;
	}
	
	params = malloc (sizeof (struct timer_parameters));
	if (params == NULL)
	{
		fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
		return -1;
	}
	memset (params, 0 , sizeof (struct timer_parameters));
	
	params->sleep_time = usec;
	params->function_to_call = function;
	params->arg = arg;

	// create and detach the timer thread
	if (pthread_create (&thread_id, NULL, timer, (void*) params) != 0)
	{
		fprintf (stderr, "Can't create a thread to handle the timer\n");
		return -1;
	}
	pthread_detach (thread_id);
	
	return 0;
}
