/**
 * @file Vector3f.h
 *
 * math Vector3f
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Vector3f.h"


// constructor
int Vector3f_init_zero (Vector3f *v)
{
	return Vector_init_zero (v, VECTOR3F_ROWS);
}

int Vector3f_init_default (Vector3f *v)
{
	return Vector_init_zero (v, VECTOR3F_ROWS);
}

int Vector3f_init_float_pointer (Vector3f *v, float *data)
{
	return Vector_init_float_pointer (v, VECTOR3F_ROWS, data);
}

int Vector3f_init_components (Vector3f *v, float x, float y, float z)
{
	return Vector_init_three_elements (v, x, y, z);
}

// copy constructor
int Vector3f_init_Vector3f (Vector3f *v, Vector3f *right)
{
	CHECK_VECTOR3F(right);
	
	return Vector_init_Vector (v, right);
}

int Vector3f_compare (Vector3f *v, Vector3f *right)
{
	CHECK_VECTOR3F(v);
	CHECK_VECTOR3F(right);
	
	return Vector_compare (v, right);
}


int Vector3f_set_all (Vector3f *v, float value)
{
	CHECK_VECTOR3F(v);
	
	return Vector_init_three_elements (v, value, value, value);
}


int Vector3f_getX (Vector3f *v, float *result)
{
	CHECK_VECTOR3F(v);
	
	return Vector_get_data (v, result, 0);
}

int Vector3f_getY (Vector3f *v, float *result)
{
	CHECK_VECTOR3F(v);
	
	return Vector_get_data (v, result, 1);
}

int Vector3f_getZ (Vector3f *v, float *result)
{
	CHECK_VECTOR3F(v);
	
	return Vector_get_data (v, result, 2);
}

int Vector3f_setX (Vector3f *v, float value)
{
	CHECK_VECTOR3F(v);
	
	return Vector_set_data (v, value, 0);
}

int Vector3f_setY (Vector3f *v, float value)
{
	CHECK_VECTOR3F(v);
	
	return Vector_set_data (v, value, 1);
}

int Vector3f_setZ (Vector3f *v, float value)
{
	CHECK_VECTOR3F(v);
	
	return Vector_set_data (v, value, 2);
}


int Vector3f_add_float (Vector3f *v, float value)
{
	CHECK_VECTOR3F(v);
	
	return Vector_add_float (v, value);
}

int Vector3f_sub_float (Vector3f *v, float value)
{
	CHECK_VECTOR3F(v);
	
	return Vector_sub_float (v, value);
}


int Vector3f_mul_float (Vector3f *v, float value)
{
	CHECK_VECTOR3F(v);
	
	return Vector_mul_float (v, value);
}


int Vector3f_div_float (Vector3f *v, float value)
{
	CHECK_VECTOR3F(v);
	
	return Vector_div_float (v, value);
}


int Vector3f_add_Vector3f (Vector3f *v, Vector3f *right)
{
	CHECK_VECTOR3F(v);
	CHECK_VECTOR3F(right);
	
	return Vector_add_Vector (v, right);
}

int Vector3f_sub_Vector3f (Vector3f *v, Vector3f *right)
{
	CHECK_VECTOR3F(v);
	CHECK_VECTOR3F(right);
		
	return Vector_sub_Vector (v, right);
}

int Vector3f_mul_Vector3f (Vector3f *v, float *result, Vector3f *right)
{
	CHECK_VECTOR3F(v);
	CHECK_VECTOR3F(right);
	
	return Vector3f_dot (v, result, right);
}

int Vector3f_cross_Vector3f (Vector3f *v, Vector3f *right)
{
	float v0, v1, v2, right0, right1, right2;
	float res0, res1, res2;
	
	CHECK_VECTOR3F(v);
	CHECK_VECTOR3F(right);
	
	MATHLIB_ASSERT (Vector3f_getX (v, &v0));
	MATHLIB_ASSERT (Vector3f_getY (v, &v1));
	MATHLIB_ASSERT (Vector3f_getZ (v, &v2));
	MATHLIB_ASSERT (Vector3f_getX (right, &right0));
	MATHLIB_ASSERT (Vector3f_getY (right, &right1));
	MATHLIB_ASSERT (Vector3f_getZ (right, &right2));
	
	res0 = v1 * right2 - v2 * right1;
	res1 = v2 * right0 - v0 * right2;
	res2 = v0 * right1 - v1 * right0;

	return Vector3f_init_components (v, res0, res1, res2);
}


int Vector3f_change_sign (Vector3f *v)
{
	CHECK_VECTOR3F(v);
	
	return Vector_change_sign (v);
}

int Vector3f_dot (Vector3f *v, float *result, Vector3f *right)
{
	CHECK_VECTOR3F(v);
	CHECK_VECTOR3F(right);

	return Vector_dot (v, result, right);
}

int Vector3f_length (Vector3f *v, float *result)
{
	CHECK_VECTOR3F(v);

	return Vector_length (v, result);
}

int Vector3f_norm (Vector3f *v, float *result)
{
	CHECK_VECTOR3F(v);

	return Vector_length (v, result);
}

int Vector3f_normalize (Vector3f *v)
{
	CHECK_VECTOR3F(v);

	return Vector_normalize (v);
}

int Vector3f_unit (Vector3f *v)
{
	CHECK_VECTOR3F(v);

	return Vector_normalize (v);
}
