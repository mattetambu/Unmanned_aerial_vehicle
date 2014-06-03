// ORB.h

#ifndef _ORB_H_
#define _ORB_H_

	/**
	 * @file uORB.h
	 * API for the uORB lightweight object broker.
	 */

	#include <semaphore.h>
	#include "../uav_library/common.h"
	#include "../uav_library/time/drv_time.h"


	#define MAX_ADVERTISERS		10
	#define MAX_SUBSCRIBERS		20


	typedef struct advertiser_data_t {
		pthread_t advertiser_pid;
	} advertiser_data_t;

	typedef struct subscriber_data_t {
		pthread_t subscriber_pid;
		uint16_t generation;		/**< last generation the subscriber has seen */
		absolute_time last_update;		/**< time of last received updated */
		absolute_time update_interval;	/**< if nonzero minimum interval between updates in usec */
		uint8_t update_reported;	/**< true if we have reported the update via poll/check */
		sem_t wake_up;
	} subscriber_data_t;

	typedef struct orb_controller_t
	{
		absolute_time last_publish;		/**< time the object was last updated */
		uint8_t _published;			/**< if nonzero, there is at least a publisher */
		uint16_t generation;		/**< last generation published */
		pthread_mutex_t _lock;
		advertiser_data_t _ad[MAX_ADVERTISERS];
		subscriber_data_t _sd[MAX_SUBSCRIBERS];
	} orb_controller_t;

	/**
	 * Object metadata.
	 */
	struct orb_metadata {
		const char *o_name;		/**< unique object name */
		const size_t o_size;		/**< object size */
		void *topic_s;			/**< pointer to the orb topic structure */
		bool_t free_publish;	/**< if true no strict pid control will be done for advertisers */
		orb_controller_t *obj_controller;	/**< object controller */
	};

	typedef const struct orb_metadata *orb_id_t;
	typedef int32_t	orb_advert_t;
	typedef int32_t	orb_subscr_t;
	
	
	/**
	 * Generates a pointer to the uORB metadata structure for
	 * a given topic.
	 *
	 * The topic must have been declared previously in scope
	 * with ORB_DECLARE().
	 *
	 * @param _name		The name of the topic.
	 */
	#define ORB_ID(_name)		&__orb_##_name

	/**
	 * Declare (prototype) the uORB metadata for a topic.
	 *
	 * Note that optional topics are declared weak; this allows a potential
	 * subscriber to attempt to subscribe to a topic that is not known to the
	 * system at runtime.  The ORB_ID() macro will return NULL/nullptr for
	 * such a topic, and attempts to advertise or subscribe to it will
	 * return -1/ENOENT (see below).
	 *
	 * @param _name		The name of the topic.
	 */
	# define ORB_DECLARE(_name)		\
		typedef struct _name##_s _name##_s;	\
		struct _name##_s __##_name;	\
		extern struct orb_metadata __orb_##_name

	# define ORB_DECLARE_OPTIONAL(_name)	\
		typedef struct _name##_s _name##_s;	\
		struct _name##_s __##_name;	\
		extern struct orb_metadata __orb_##_name __attribute__((weak))

	# define ORB_DECLARE_MANY(_name, _struct)	\
		typedef _struct _name##_s;	\
		_struct __##_name;	\
		extern struct orb_metadata __orb_##_name


	/**
	 * Define (instantiate) the uORB metadata for a topic.
	 *
	 * The uORB metadata is used to help ensure that updates and
	 * copies are accessing the right data.
	 *
	 * Note that there must be no more than one instance of this macro
	 * for each topic.
	 *
	 * @param _name		The name of the topic.
	 * @param _struct	The structure the topic provides.
	 */
	#define ORB_DEFINE(_name, _struct)	\
		struct orb_metadata __orb_##_name = {	\
			#_name,				\
			sizeof(_struct),	\
			(void *) &__##_name,	\
			0,					\
			NULL				\
		}; struct hack

	#define ORB_DEFINE_FREE_PUBLISH(_name, _struct)	\
		struct orb_metadata __orb_##_name = {	\
			#_name,				\
			sizeof(_struct),	\
			(void *) &__##_name,	\
			1,					\
			NULL				\
		}; struct hack

	
	/* function prototypes */
	void system_orb_init ();
	orb_advert_t orb_advertise (struct orb_metadata *meta);
	orb_subscr_t orb_subscribe (struct orb_metadata *meta);
	int orb_unadvertise (const struct orb_metadata *meta, orb_advert_t advertiser, pthread_t pid);
	int orb_unsubscribe (const struct orb_metadata *meta, orb_subscr_t subscriber, pthread_t pid);
	int orb_publish (const struct orb_metadata *meta, orb_advert_t advertiser, const void *data);
	int orb_copy (const struct orb_metadata *meta, orb_subscr_t subscriber, void *buffer);
	int orb_check (const struct orb_metadata *meta, orb_subscr_t subscriber);
	int orb_poll (const struct orb_metadata *meta, orb_subscr_t subscriber, absolute_time usec_wait_time);
	int orb_wait (const struct orb_metadata *meta, orb_subscr_t subscriber);
	int orb_stat (const struct orb_metadata *meta, orb_subscr_t subscriber, absolute_time *last_publish);
	int orb_set_interval (const struct orb_metadata *meta, orb_subscr_t subscriber, absolute_time interval);
	
	/* global variables */
	// empty	
	
#endif
