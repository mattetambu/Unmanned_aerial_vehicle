/**
 * @file Quaternion.h
 *
 * math Quaternion
 */


#ifndef	_MATH_QUATERNION_H_
#define	_MATH_QUATERNION_H_

	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>

	#include "Vector.h"
	#include "Matrix.h"


	#define QUATERNION_ROWS		4
	#define CHECK_QUATERNION(q)	if (!q || q->rows != QUATERNION_ROWS) return -1;

	
	// constructor
	int Quaternion_init_zero (Quaternion *q);
	int Quaternion_init_default (Quaternion *q);
	int Quaternion_init_float_pointer (Quaternion *q, float *data);
	int Quaternion_init_components (Quaternion *q, float a, float b, float c, float d);
	/*int Quaternion_init_Vector (Quaternion *q, Vector *v);*/
	int Quaternion_init_EulerAngles (Quaternion *q, EulerAngles *e);
	int Quaternion_init_Dcm (Quaternion *q, Dcm *d);
	
	// copy constructor
	int Quaternion_init_Quaternion (Quaternion *q, Quaternion *right);
	
	int Quaternion_compare (Quaternion *q, Quaternion *right);
	int Quaternion_set_all (Quaternion *q, float value);
	int Quaternion_getA (Quaternion *q, float *result);
	int Quaternion_getB (Quaternion *q, float *result);
	int Quaternion_getC (Quaternion *q, float *result);
	int Quaternion_getD (Quaternion *q, float *result);
	int Quaternion_setA (Quaternion *q, float value);
	int Quaternion_setB (Quaternion *q, float value);
	int Quaternion_setC (Quaternion *q, float value);
	int Quaternion_setD (Quaternion *q, float value);

	int Quaternion_add_float (Quaternion *q, float value);
	int Quaternion_sub_float (Quaternion *q, float value);
	int Quaternion_mul_float (Quaternion *q, float value);
	int Quaternion_div_float (Quaternion *q, float value);
	int Quaternion_add_Quaternion (Quaternion *q, Quaternion *right);
	int Quaternion_sub_Quaternion (Quaternion *q, Quaternion *right);
	
	int Quaternion_change_sign (Quaternion *q);
	int Quaternion_dot (Quaternion *v, float *result, Quaternion *right);
	int Quaternion_length (Quaternion *v, float *result);
	int Quaternion_norm (Quaternion *v, float *result);
	int Quaternion_normalize (Quaternion *v);
	int Quaternion_unit (Quaternion *v);
	int Quaternion_derivative (Quaternion *q, Vector *v);


#endif
