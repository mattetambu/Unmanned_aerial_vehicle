/**
 * @file Vector3f.h
 *
 * math Vector3f
 */


#ifndef	_MATH_VECTOR3F_H_
#define	_MATH_VECTOR3F_H_

	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>
	
	#include "Vector.h"


	#define VECTOR3F_ROWS		3
	#define CHECK_VECTOR3F(v)	if (!v || v->rows != VECTOR3F_ROWS) return -1;

	
	// constructor
	int Vector3f_init_zero (Vector3f *v);
	int Vector3f_init_default (Vector3f *v);
	int Vector3f_init_float_pointer (Vector3f *v, float *data);
	int Vector3f_init_components (Vector3f *v, float x, float y, float z);
	
	// copy constructor
	int Vector3f_init_Vector3f (Vector3f *v, Vector3f *right);
	
	int Vector3f_compare (Vector3f *v, Vector3f *right);
	int Vector3f_set_all (Vector3f *v, float value);
	int Vector3f_getX (Vector3f *v, float *result);
	int Vector3f_getY (Vector3f *v, float *result);
	int Vector3f_getZ (Vector3f *v, float *result);
	int Vector3f_setX (Vector3f *v, float value);
	int Vector3f_setY (Vector3f *v, float value);
	int Vector3f_setZ (Vector3f *v, float value);

	int Vector3f_add_float (Vector3f *v, float value);
	int Vector3f_sub_float (Vector3f *v, float value);
	int Vector3f_mul_float (Vector3f *v, float value);
	int Vector3f_div_float (Vector3f *v, float value);
	int Vector3f_add_Vector3f (Vector3f *v, Vector3f *right);
	int Vector3f_sub_Vector3f (Vector3f *v, Vector3f *right);
	int Vector3f_mul_Vector3f (Vector3f *v, float *result, Vector3f *right);
	int Vector3f_emul_Vector3f (Vector3f *v, Vector3f *result, Vector3f *right);
	int Vector3f_ediv_Vector3f (Vector3f *v, Vector3f *result, Vector3f *right);
	int Vector3f_cross_Vector3f (Vector3f *v, Vector3f *right);

	int Vector3f_change_sign (Vector3f *v);
	int Vector3f_dot (Vector3f *v, float *result, Vector3f *right);
	int Vector3f_length (Vector3f *v, float *result);
	int Vector3f_norm (Vector3f *v, float *result);
	int Vector3f_normalize (Vector3f *v);
	int Vector3f_unit (Vector3f *v);
	
#endif
