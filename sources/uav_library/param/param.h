/**
 * @file param.h
 *
 * Global parameter store.
 *
 * Note that a number of API members are marked const or pure; these
 * assume that the set of parameters cannot change, or that a parameter
 * cannot change type or size over its lifetime.  If any of these assumptions
 * are invalidated, the attributes should be re-evaluated.
 */

#ifndef __PARAM_PARAM_H
#define __PARAM_PARAM_H

	#include <stdint.h>
	#include <sys/types.h>
	#include "../common.h"

	/**
	 * Parameter types.
	 */
	typedef enum {
		/* globally-known parameter types */
		PARAM_TYPE_INT32 = 0,
		PARAM_TYPE_FLOAT,

		/* structure parameters; size is encoded in the type value */
		PARAM_TYPE_STRUCT = 100,
		PARAM_TYPE_STRUCT_MAX = 16384 + PARAM_TYPE_STRUCT,

		PARAM_TYPE_UNKNOWN = 0xffff
	} param_type_t;


	/*
	// define an int32 parameter
	#define PARAM_DEFINE_INT(_name, _default)	\
		static const __attribute__((used, section("__param"))) struct param_info_s __param__##_name = {	\
			.name = (const char *) #_name,		\
			.last_modification_time = 0,		\
			.type = PARAM_TYPE_INT32,			\
			.val.i = _default,					\
			.default_val.i = _default			\
		};

	// define a float parameter
	#define PARAM_DEFINE_FLOAT(_name, _default)	\
		static const __attribute__((used, section("__param"))) struct param_info_s __param__##_name = {	\
			.name = (const char *) #_name,		\
			.last_modification_time = 0,		\
			.type = PARAM_TYPE_FLOAT,			\
			.val.i = _default,					\
			.default_val.i = _default			\
		};

	// define a parameter that points to a structure
	#define PARAM_DEFINE_STRUCT(_name, _default)	\
		static const __attribute__((used, section("__param"))) struct param_info_s __param__##_name = {	\
			.name = (const char *) #_name,		\
			.last_modification_time = 0,		\
			.type = PARAM_TYPE_STRUCT,			\
			.val.i = &_default,					\
			.default_val.i = &_default			\
		};
	 */


	/**
	 * Parameter handle.
	 *
	 * Parameters are represented by parameter handles, which can
	 * be obtained by looking up (or creating?) parameters.
	 */
	typedef unsigned long param_t;


	/**
	 * Handle returned when a parameter cannot be found.
	 */
	#define PARAM_INVALID	((unsigned long)0xffffffff)


	/* function prototypes */
	int param_init ();
	int param_define_int (const char *name, int val);
	int param_define_float (const char *name, float val);
	int param_define_struct (const char *name, const void *val, uint32_t ssize);
	param_t param_find(const char *name);
	unsigned param_count();
	int param_get_index(param_t param);
	int param_size(param_t param);
	const char * param_name(param_t param);
	param_type_t param_type(param_t param);
	bool_t param_value_is_default(param_t param);
	int param_get(param_t param, void *val);
	int param_set(param_t param, const void *val);
	int param_reset(param_t param);
	//void param_foreach (void (*func)(void *arg, param_t param), void *arg, bool_t only_changed);

	/* global variables */
	// empty

#endif
