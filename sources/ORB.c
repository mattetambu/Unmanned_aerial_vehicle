// ORB.c

#include "ORB.h"
#include <pthread.h>


pthread_mutex_t orb_init_lock;
int inizialized = 0;

static inline void ORB_LOCK_TOPIC (pthread_mutex_t *lock) {
	do {} while (pthread_mutex_lock (lock) != 0);
}

static inline void ORB_UNLOCK_TOPIC (pthread_mutex_t *lock) {
	pthread_mutex_unlock (lock);
}
	
#define ORB_UNLOCK_TOPIC_AND_RETURN(_lock, _val)	\
	ORB_UNLOCK_TOPIC (_lock);	\
	return _val;


void system_orb_init ()
{
	if (inizialized)
		return;

	pthread_mutex_init (&orb_init_lock, NULL);
	inizialized = 1;
}


void *
wake_up_subscriber (void *arg)
{
	sem_t *sem = (sem_t *) arg;
	int sem_value = 0;

	if (sem != NULL)
		sem_getvalue (sem, &sem_value);
	
	if (sem_value != 0)
		sem_post (sem);
	
	return NULL;
}

int
orb_controller_init (struct orb_metadata *meta)
{
	if (!meta)
	{
		fprintf (stderr, "Can't advertise, the topic does not exist\n");
		errno = EINVAL;
		return -1;
	}
	
	if (!inizialized)
	{
		fprintf (stderr, "The orb must be initialized first by calling orv_init()\n");
		return -1;
	}
	
	// get the lock for the critical section
	ORB_LOCK_TOPIC (&orb_init_lock);
	
	if (!meta->obj_controller)
	{
		meta->obj_controller = malloc (sizeof(orb_controller_t));
		memset (meta->obj_controller, 0, sizeof(orb_controller_t));
		pthread_mutex_init (&meta->obj_controller->_lock, NULL);
		if (!meta->obj_controller->_lock)
		{
			fprintf (stderr, "Failed to initialize the orb mutex\n");
			
			// release the lock on the topic and return the error
			ORB_UNLOCK_TOPIC_AND_RETURN (&orb_init_lock, -1);
		}
	}
	
	// release the lock
	ORB_UNLOCK_TOPIC (&orb_init_lock);
	
	return 0;
}

int
orb_check_subscriber (const struct orb_metadata *meta, orb_subscr_t subscriber, const char *op)
{
	int i = (int) subscriber;
	
	/* check if this topic has been published yet, if not bail out */
	if (!meta)
	{
		fprintf (stderr, "Can't %s, the topic does not exist\n", op);
		errno = EINVAL;
		return -1;
	}
	
	if (!meta->obj_controller)
	{
		fprintf (stderr, "Can't %s, topic not initialized\n", op);
		errno = EINVAL;
		return -1;
	}

	if (i > MAX_SUBSCRIBERS)
	{
		fprintf (stderr, "Can't %s, invalid subcriber identifier\n", op);
		errno = EINVAL;
		return -1;
	}
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	if (meta->obj_controller->_sd[i].subscriber_pid == 0)
	{
		fprintf (stderr, "Can't %s, need first to be subcribed to the topic\n", op);
		
		// release the lock on the topic and return the error
		ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, -1);
	}
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}

int
orb_check_advertiser (const struct orb_metadata *meta, orb_advert_t advertiser, const char *op)
{
	int i = (int) advertiser;
	
	/* check if this topic has been published yet, if not bail out */
	if (!meta)
	{
		fprintf (stderr, "Can't %s, the topic does not exist\n", op);
		errno = EINVAL;
		return -1;
	}
	
	if (!meta->obj_controller)
	{
		fprintf (stderr, "Can't %s, topic not initialized\n", op);
		errno = EINVAL;
		return -1;
	}

	if (i > MAX_ADVERTISERS)
	{
		fprintf (stderr, "Can't %s, invalid advertiser identifier\n", op);
		errno = EINVAL;
		return -1;
	}
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	if (meta->obj_controller->_ad[i].advertiser_pid == 0)
	{
		fprintf (stderr, "Can't %s, need first to advertise the topic\n", op);
		
		// release the lock on the topic and return the error
		ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, -1);
	}
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}

orb_advert_t
orb_advertise (struct orb_metadata *meta, pid_t pid)
{
	int i;
	
	if (orb_controller_init (meta) < 0)
	{
		fprintf (stderr, "Can't initialize the topic, it does not exist\n");
		errno = EINVAL;
		return -1;
	}
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	for (i = 0; i < MAX_ADVERTISERS; i++)
		if (meta->obj_controller->_ad[i].advertiser_pid == pid)
		{
			// that's not an error - already advertised
			// release the lock on the topic and return the advertiser
			ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, (orb_advert_t) i);
		}
		
	for (i = 0; i < MAX_ADVERTISERS; i++)
		if (meta->obj_controller->_ad[i].advertiser_pid == 0)
			break;
	
	if (i == MAX_ADVERTISERS)
	{
		fprintf (stderr, "Can't advertise, too many advertisers\n");
		
		// release the lock on the topic and return the error
		ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, -1);
	}
	
	meta->obj_controller->_published = 1;
	meta->obj_controller->_ad[i].advertiser_pid = pid;
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return (orb_advert_t) i;
}

orb_subscr_t
orb_subscribe (struct orb_metadata *meta, pid_t pid)
{
	int i;

	if (orb_controller_init (meta) < 0)
	{
		fprintf (stderr, "Can't initialize the topic, it does not exist\n");
		errno = EINVAL;
		return -1;
	}
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	for (i = 0; i < MAX_SUBSCRIBERS; i++)
		if (meta->obj_controller->_sd[i].subscriber_pid == pid)
		{
			// that's not an error - already subscribed
			// release the lock on the topic and return the subcriber
			ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, (orb_advert_t) i);
		}
		
	for (i = 0; i < MAX_SUBSCRIBERS; i++)
		if (meta->obj_controller->_sd[i].subscriber_pid == 0)
			break;
	
	if (i == MAX_SUBSCRIBERS)
	{
		fprintf (stderr, "Can't subscribe, too many subscribers\n");
		
		// release the lock on the topic and return the error
		ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, -1);
	}
	
	meta->obj_controller->_sd[i].subscriber_pid = pid;
	sem_init (&meta->obj_controller->_sd[i].wake_up, 0, 0);
	if (!meta->obj_controller->_sd[i].wake_up)
	{
		fprintf (stderr, "Failed to initialize the orb semaphore\n");
		
		// release the lock on the topic and return the error
		ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, -1);
	}

	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return (orb_advert_t) i;
}

int
orb_unadvertise (const struct orb_metadata *meta, orb_advert_t advertiser)
{
	int i = 0;
	
	if (orb_check_advertiser (meta, advertiser, "unadvertise") < 0)
		// already printed an error message
		return -1;
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	memset ((void *) &meta->obj_controller->_ad[advertiser], 0, sizeof (advertiser_data_t));
	
	for (i = 0; i < MAX_ADVERTISERS; i++)
		if (meta->obj_controller->_ad[i].advertiser_pid != 0)
			break;
	
	if (i == MAX_ADVERTISERS)
		meta->obj_controller->_published = 0;
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}

int
orb_unsubscribe (const struct orb_metadata *meta, orb_subscr_t subscriber)
{
	if (orb_check_subscriber (meta, subscriber, "unsubscribe") < 0)
		// already printed an error message
		return -1;
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	sem_destroy (&meta->obj_controller->_sd[subscriber].wake_up);
	memset ((void *) &meta->obj_controller->_sd[subscriber], 0, sizeof (subscriber_data_t));
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}

int
orb_publish (const struct orb_metadata *meta, orb_advert_t advertiser, const void *data)
{
	int i;
	
	if (orb_check_advertiser (meta, advertiser, "publish") < 0)
		// already printed an error message
		return -1;
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	meta->obj_controller->generation++;
	meta->obj_controller->last_publish = get_absolute_time ();
	
	// export the data in the pointed structure
	memcpy (meta->topic_s, data, meta->o_size);

	// wake up all the waiting subscribers
	for (i = 0; i < MAX_SUBSCRIBERS; i++)
		if (meta->obj_controller->_sd[i].subscriber_pid &&
			!meta->obj_controller->_sd[i].update_reported)
			wake_up_subscriber ((void *) &meta->obj_controller->_sd[i].wake_up);
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}

int
orb_copy (const struct orb_metadata *meta, orb_subscr_t subscriber, void *buffer)
{
	if (orb_check_subscriber (meta, subscriber, "copy") < 0)
		// already printed an error message
		return -1;
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	meta->obj_controller->_sd[subscriber].last_update = get_absolute_time ();
	meta->obj_controller->_sd[subscriber].generation = meta->obj_controller->generation;
	meta->obj_controller->_sd[subscriber].update_reported = 0;
	
	// inport the data and return in the pointed buffer
	memcpy (buffer, (const void *) meta->topic_s, meta->o_size);

	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}

int
orb_check (const struct orb_metadata *meta, orb_subscr_t subscriber)
{
	absolute_time delta_time;
	
	if (orb_check_subscriber (meta, subscriber, "check for updates") < 0)
		// already printed an error message
		return -1;
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	if (!meta->obj_controller->_published)
	{
		fprintf (stderr, "Can't check for updates, no advertiser registered for the topic\n");
		errno = EINVAL;
		
		// release the lock on the topic and return the error
		ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, -1);
	}
	
	/*
	 * If the subscriber's generation count matches the update generation
	 * count, there has been no update from their perspective; if they
	 * don't match then we might have a visible update.
	 */
	if (meta->obj_controller->generation != meta->obj_controller->_sd[subscriber].generation) {
		/*
		 * Handle non-rate-limited subscribers.
		 */
		if (meta->obj_controller->_sd[subscriber].update_interval == 0)
		{
			// release the lock on the topic and return
			ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, 1);
		}			

		delta_time = absolute_elapsed_time (&meta->obj_controller->_sd[subscriber].last_update);
		if (delta_time > meta->obj_controller->_sd[subscriber].update_interval)
		{
			meta->obj_controller->_sd[subscriber].update_reported = 0;
			
			// release the lock on the topic and return
			ORB_UNLOCK_TOPIC_AND_RETURN (&meta->obj_controller->_lock, 1);		
		}

		// the update is present but does not have to be notified
		if (!meta->obj_controller->_sd[subscriber].update_reported)
		{
			set_timeout (delta_time, wake_up_subscriber, (void *) &meta->obj_controller->_sd[subscriber].wake_up);
			meta->obj_controller->_sd[subscriber].update_reported = 1;			
		}
	}
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}

int
orb_poll (const struct orb_metadata *meta, orb_subscr_t subscriber, absolute_time usec_wait_time)
{
	int check_return_value = orb_check (meta, subscriber);
	int timedwait_return_value;
	struct timespec max_time;
	
	if (check_return_value < 0)
	{
		fprintf (stderr, "Failed to poll the topic\n");
		return -1;
	}
	
	if (check_return_value > 0)
		// a valid update is already available, no need to wait
		return 1;
	
	if (usec_wait_time < 0)
	{
		fprintf (stderr, "Poll deadline must be grater than 0\n");
		errno = EINVAL;
		return -1;
	}
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	absolute_time_to_timespec (get_absolute_time () + usec_wait_time, &max_time);
	timedwait_return_value = sem_timedwait (&meta->obj_controller->_sd[subscriber].wake_up, (const struct timespec *) &max_time);
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return (!timedwait_return_value)? 1 : 0;
}

int
orb_wait (const struct orb_metadata *meta, orb_subscr_t subscriber)
{
	int check_return_value = orb_check (meta, subscriber);
	
	if (check_return_value < 0)
	{
		fprintf (stderr, "Failed to wait fot the topic\n");
		return -1;
	}
	
	if (check_return_value > 0)
		// a valid update is already available, no need to wait
		return 1;
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	// wait for a new update
	sem_wait (&meta->obj_controller->_sd[subscriber].wake_up);
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 1;
}

int
orb_stat (const struct orb_metadata *meta, orb_subscr_t subscriber, uint32_t *last_publish)
{
	if (orb_check_subscriber (meta, subscriber, "get orb stat") < 0)
		// already printed an error message
		return -1;
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	*last_publish = meta->obj_controller->last_publish;
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}
	
int
orb_set_interval (const struct orb_metadata *meta, orb_subscr_t subscriber, absolute_time interval)
{
	if (orb_check_subscriber (meta, subscriber, "set interval") < 0)
		// already printed an error message
		return -1;
	
	// obtain the lock on the topic for the critical section
	ORB_LOCK_TOPIC (&meta->obj_controller->_lock);
	
	meta->obj_controller->_sd[subscriber].update_interval = interval;
	
	// release the lock on the topic
	ORB_UNLOCK_TOPIC (&meta->obj_controller->_lock);
	
	return 0;
}

