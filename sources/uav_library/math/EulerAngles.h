/**
 * @file EulerAngles.h
 *
 * math EulerAngles
 */


#ifndef	_MATH_EULERANGLES_H_
#define	_MATH_EULERANGLES_H_

	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>

	#include "Vector.h"
	#include "Matrix.h"


	#define EULERANGLES_ROWS		3
	#define CHECK_EULERANGLES(e)	if (!e || e->rows != EULERANGLES_ROWS) return -1;

	
	// constructor
	int EulerAngles_init_zero (EulerAngles *e);
	int EulerAngles_init_default (EulerAngles *e);
	int EulerAngles_init_float_pointer (EulerAngles *e, float *data);
	int EulerAngles_init_components (EulerAngles *e, float x, float y, float z);
	int EulerAngles_init_Dcm (EulerAngles *e, Dcm *d);
	int EulerAngles_init_Quaternion (EulerAngles *e, Quaternion *q);
	
	// copy constructor
	int EulerAngles_init_EulerAngles (EulerAngles *e, EulerAngles *right);
	
	int EulerAngles_compare (EulerAngles *e, EulerAngles *right);
	int EulerAngles_set_all (EulerAngles *e, float value);
	int EulerAngles_getPhi (EulerAngles *e, float *result);
	int EulerAngles_getTheta (EulerAngles *e, float *result);
	int EulerAngles_getPsi (EulerAngles *e, float *result);
	int EulerAngles_setPhi (EulerAngles *e, float value);
	int EulerAngles_setTheta (EulerAngles *e, float value);
	int EulerAngles_setPsi (EulerAngles *e, float value);

	int EulerAngles_add_float (EulerAngles *e, float value);
	int EulerAngles_sub_float (EulerAngles *e, float value);
	int EulerAngles_mul_float (EulerAngles *e, float value);
	int EulerAngles_div_float (EulerAngles *e, float value);
	int EulerAngles_add_EulerAngles (EulerAngles *e, EulerAngles *right);
	int EulerAngles_sub_EulerAngles (EulerAngles *e, EulerAngles *right);
	
	int EulerAngles_change_sign (EulerAngles *e);
	int EulerAngles_dot (EulerAngles *v, float *result, EulerAngles *right);
	int EulerAngles_length (EulerAngles *v, float *result);
	int EulerAngles_norm (EulerAngles *v, float *result);
	int EulerAngles_normalize (EulerAngles *v);
	int EulerAngles_unit (EulerAngles *v);

#endif
