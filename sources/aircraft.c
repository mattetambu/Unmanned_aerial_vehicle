// aircraft.c

#include "aircraft.h"


aircraft_t *aircraft = NULL;			// The aircraft

int aircraft_init ()
{
	aircraft = malloc (sizeof(aircraft_t));
	if (!aircraft)
	{
		fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
		return -1;
	}
	memset (aircraft, 0, sizeof (aircraft_t));
	
	return 0;
}

void aircraft_destroy ()
{
	if (!aircraft)
		return;
		
	free (aircraft);
}
