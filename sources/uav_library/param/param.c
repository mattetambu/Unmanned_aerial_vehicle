/**
 * @file param.c
 *
 * Global parameter store.
 *
 * Note that it might make sense to convert this into a driver.  That would
 * offer some interesting options regarding state for e.g. ORB advertisements
 * and background parameter saving.
 */

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/stat.h>

#include "param.h"
#include "../common.h"
#include "../time/drv_time.h"

#include "../../ORB/ORB.h"
#include "../../ORB/topics/parameter_update.h"



/**
 * Parameter value union.
 */
typedef union  {
	void *p;
	int32_t i;
	float f;
} param_value_u;


#define	PARAM_NAME_MAX_LENGTH		15

/**
 * Static parameter definition structure.
 *
 * This is normally not used by user code; see the PARAM_DEFINE macros
 * instead.
 */
struct param_info_s {
	char name[PARAM_NAME_MAX_LENGTH+1];
	absolute_time last_modification_time;
	param_type_t type;

	param_value_u val;
	param_value_u default_val;
};

/**
 * Array of static parameter info.
 */
#define	PARAM_INFO_MAX_COUNT		100
param_t insert_param_count = 0;		// assert <= PARAM_INFO_MAX_COUNT
struct param_info_s params_array [PARAM_INFO_MAX_COUNT];


/** parameter update topic handle */
pthread_mutex_t param_mutex;
bool_t param_mutex_initialized = 0 /* false */;


/** lock the parameter store */
inline void param_lock(void)
{
	//do {} while (pthread_mutex_lock(&param_mutex) != 0);
}

/** unlock the parameter store */
inline void param_unlock(void)
{
	//pthread_mutex_unlock(&param_mutex);
}

/** assert that the parameter store is locked */
inline void param_assert_locked(void)
{
	/* XXX */
}


int param_init ()
{
	if (param_mutex_initialized)
		return 0;

	memset (params_array, 0, sizeof(params_array));
	if (pthread_mutex_init (&param_mutex, NULL) == -1)
		return -1;

	param_mutex_initialized = 1;
	return 0;
}


void param_notify_changes()
{
	orb_advert_t param_adv = 0; // param_update is free published (no need of subscription)
	struct parameter_update_s param_update = { .timestamp = get_absolute_time() };

	orb_publish(ORB_ID(parameter_update), param_adv, &param_update);
}


unsigned param_count()
{
	return insert_param_count;
}

/**
 * Test whether a param_t is value.
 *
 * @param param			The parameter handle to test.
 * @return			True if the handle is valid.
 */
bool_t handle_in_range(param_t param)
{
	return (param >= 0 && param < insert_param_count);
}


int param_define (const char *name, param_type_t p_type, const void *val)
{
	int param_valid = 0;
	param_t param;


	if (insert_param_count == PARAM_INFO_MAX_COUNT)
	{
		fprintf (stderr, "Impossible to define a new parameter (limit reached)\n");
		return PARAM_INVALID;
	}

	if (strlen(name) > PARAM_NAME_MAX_LENGTH)
	{
		fprintf (stderr, "Parameter name is not valid, must not exceed 15 characters\n");
		return PARAM_INVALID;
	}

	/* perform a linear search of the known parameters */
	for (param = 0; param < insert_param_count; param++) {
		if (!strcmp(params_array[param].name, name))
			{
				fprintf (stderr, "Parameter \'%s\' already defined\n", name);
				return param;
			}
	}

	param_lock();

	/* update the changed value */
	switch (p_type) {
		case PARAM_TYPE_INT32:
			params_array[insert_param_count].val.i = *(int32_t *)val;
			params_array[insert_param_count].default_val.i = *(int32_t *)val;
			param_valid = 1;
			break;

		case PARAM_TYPE_FLOAT:
			params_array[insert_param_count].val.f = *(float *)val;
			params_array[insert_param_count].default_val.f = *(float *)val;
			param_valid = 1;
			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			if (params_array[insert_param_count].val.p == NULL) {
				params_array[insert_param_count].val.p = malloc(p_type - PARAM_TYPE_STRUCT);

				if (params_array[insert_param_count].val.p == NULL) {
					fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
					break;
				}
			}

			if (params_array[insert_param_count].default_val.p == NULL) {
				params_array[insert_param_count].default_val.p = malloc(p_type - PARAM_TYPE_STRUCT);

				if (params_array[insert_param_count].default_val.p == NULL) {
					fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
					break;
				}
			}

			memcpy(params_array[insert_param_count].val.p, val, (p_type - PARAM_TYPE_STRUCT));
			memcpy(params_array[insert_param_count].default_val.p, val, (p_type - PARAM_TYPE_STRUCT));
			param_valid = 1;
			break;

		default:
			break;
	}


	if (param_valid)
	{
		strcpy (params_array[insert_param_count].name, name);
		params_array[insert_param_count].type = p_type;
		params_array[insert_param_count].last_modification_time = get_absolute_time();
		insert_param_count++;
		param_notify_changes();
	}

	param_unlock();

	return (param_valid)? 0 : PARAM_INVALID;
}


int param_define_int (const char *name, int val)
{
	int value = val;

	return param_define (name, PARAM_TYPE_INT32, (const void *) &value);
}

int param_define_float (const char *name, float val)
{
	float value = val;

	return param_define (name, PARAM_TYPE_FLOAT, (const void *) &value);
}

int param_define_struct (const char *name, const void *val, uint32_t ssize)
{
	return param_define (name, PARAM_TYPE_STRUCT+ssize, (const void *) val);
}


int param_get_index(param_t param)
{
	if (handle_in_range(param))
		return (unsigned)param;

	return PARAM_INVALID;
}

const char * param_name(param_t param)
{
	if (handle_in_range(param))
		return params_array[param].name;

	return NULL;
}

param_type_t param_type(param_t param)
{
	if (handle_in_range(param))
		return params_array[param].type;

	return PARAM_TYPE_UNKNOWN;
}

int param_size(param_t param)
{
	if (handle_in_range(param)) {
		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
		case PARAM_TYPE_FLOAT:
			return 4;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			/* decode structure size from type value */
			return param_type(param) - PARAM_TYPE_STRUCT;

		default:
			break;
		}
	}

	return PARAM_INVALID;
}


int param_find_changed(param_t param, absolute_time last_seen_time)
{
	absolute_time lm_time;

	if (!handle_in_range(param))
		return PARAM_INVALID;

	lm_time = params_array[param].last_modification_time;
	return (lm_time == 0) ? 0 : (lm_time > last_seen_time);
}

bool_t param_value_is_default(param_t param)
{
	if (!handle_in_range(param))
		return PARAM_INVALID;

	return (params_array[param].last_modification_time == 0) ? 1 : 0;
}

param_t param_find(const char *name)
{
	param_t param;

	if (strlen(name) > PARAM_NAME_MAX_LENGTH)
	{
		fprintf (stderr, "Parameter name is not valid, must not exceed 15 characters\n");
		return PARAM_INVALID;
	}

	/* perform a linear search of the known parameters */
	for (param = 0; param < insert_param_count; param++) {
		if (!strcmp(params_array[param].name, name))
			return param;
	}

	/* not found */
	return PARAM_INVALID;
}


/**
 * Obtain a pointer to the storage allocated for a parameter.
 *
 * @param param			The parameter whose storage is sought.
 * @return			A pointer to the parameter value, or NULL
 *				if the parameter does not exist.
 */
const void * param_get_value_ptr(param_t param)
{
	param_value_u *v;

	param_assert_locked();

	if (!handle_in_range(param))
		return NULL;

	v = &params_array[param].val;

	return (param_type(param) == PARAM_TYPE_STRUCT) ? v->p : v;
}

int param_get(param_t param, void *val)
{
	int result = PARAM_INVALID;

	param_lock();

	const void *v = param_get_value_ptr(param);

	if (val != NULL && v != NULL) {
		memcpy(val, v, param_size(param));
		result = 0;
	}

	param_unlock();

	return result;
}


int param_set(param_t param, const void *val)
{
	int param_changed = 0;
	param_value_u *param_val;

	param_lock();

	if (handle_in_range(param))
	{
		param_val = (param_value_u *) param_get_value_ptr(param);

		/* update the changed value */
		switch (param_type(param)) {
			case PARAM_TYPE_INT32:
				param_val->i = *(int32_t *)val;
				param_changed = 1;
				break;

			case PARAM_TYPE_FLOAT:
				param_val->f = *(float *)val;
				param_changed = 1;
				break;

			case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
				if (param_val->p == NULL) {
					param_val->p = malloc(param_size(param));

					if (param_val->p == NULL) {
						fprintf (stderr, "Impossible to allocate memory (malloc error)\n");
						break;
					}
				}

				memcpy(param_val->p, val, param_size(param));
				param_changed = 1;
				break;

			default:
				break;
		}
	}

	if (param_changed)
	{
		params_array[param].last_modification_time = get_absolute_time();
		param_notify_changes();
	}

	param_unlock();

	return (param_changed)? 0 : PARAM_INVALID;
}

int param_reset(param_t param)
{
	int result = PARAM_INVALID;

	if (handle_in_range(param))
	{
		switch (param_type(param)) {
			case PARAM_TYPE_INT32:
				result = param_set(param, (const void *) &params_array[param].default_val.i);
				break;

			case PARAM_TYPE_FLOAT:
				result = param_set(param, (const void *) &params_array[param].default_val.f);
				break;

			case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
				result = param_set(param, (const void *) params_array[param].default_val.p);
				break;

			default:
				break;
		}
	}

	return result;
}

/*
void param_foreach(void (*func)(void *arg, param_t param), void *arg, bool_t only_changed)
{
	param_t	param;

	for (param = 0; handle_in_range(param); param++) {

		// if requested, skip unchanged values
		if (only_changed && (param_find_changed(param) == NULL))
			continue;

		func(arg, param);
	}
}
*/
