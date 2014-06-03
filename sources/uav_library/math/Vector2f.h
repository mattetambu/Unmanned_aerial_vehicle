/**
 * @file Vector2f.h
 *
 * math Vector2f
 */


#ifndef	_MATH_VECTOR2F_H_
#define	_MATH_VECTOR2F_H_

	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>
	
	#include "Vector.h"


	#define VECTOR2F_ROWS		2
	#define CHECK_VECTOR2F(v)	if (!v || v->rows != VECTOR2F_ROWS) return -1;

	
	// constructor
	int Vector2f_init_zero (Vector2f *v);
	int Vector2f_init_default (Vector2f *v);
	int Vector2f_init_float_pointer (Vector2f *v, float *data);
	int Vector2f_init_components (Vector2f *v, float x, float y);
	
	// copy constructor
	int Vector2f_init_Vector2f (Vector2f *v, Vector2f *right);
	
	int Vector2f_compare (Vector2f *v, Vector2f *right);
	int Vector2f_set_all (Vector2f *v, float value);
	int Vector2f_getX (Vector2f *v, float *result);
	int Vector2f_getY (Vector2f *v, float *result);
	int Vector2f_setX (Vector2f *v, float value);
	int Vector2f_setY (Vector2f *v, float value);
	
	int Vector2f_add_float (Vector2f *v, float value);
	int Vector2f_sub_float (Vector2f *v, float value);
	int Vector2f_mul_float (Vector2f *v, float value);
	int Vector2f_div_float (Vector2f *v, float value);
	int Vector2f_add_Vector2f (Vector2f *v, Vector2f *right);
	int Vector2f_sub_Vector2f (Vector2f *v, Vector2f *right);	
	int Vector2f_mul_Vector2f (Vector2f *v, float *result, Vector2f *right);
	int Vector2f_emul_Vector2f (Vector3f *v, Vector3f *result, Vector3f *right);
	int Vector2f_ediv_Vector2f (Vector3f *v, Vector3f *result, Vector3f *right);
	int Vector2f_cross_Vector2f (Vector2f *v, float *result, Vector2f *right);	

	int Vector2f_change_sign (Vector2f *v);
	int Vector2f_dot (Vector2f *v, float *result, Vector2f *right);
	int Vector2f_length (Vector2f *v, float *result);
	int Vector2f_norm (Vector2f *v, float *result);
	int Vector2f_normalize (Vector2f *v);
	int Vector2f_unit (Vector2f *v);
	
#endif
