/**
 * @file Vector2f.h
 *
 * math Vector2f
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Vector2f.h"


// constructor
int Vector2f_init_zero (Vector2f *v)
{
	return Vector_init_zero (v, VECTOR2F_ROWS);
}

int Vector2f_init_default (Vector2f *v)
{
	return Vector_init_zero (v, VECTOR2F_ROWS);
}

int Vector2f_init_float_pointer (Vector2f *v, float *data)
{
	return Vector_init_float_pointer (v, VECTOR2F_ROWS, data);
}

int Vector2f_init_components (Vector2f *v, float x, float y)
{
	return Vector_init_two_elements (v, x, y);
}

// copy constructor
int Vector2f_init_Vector2f (Vector2f *v, Vector2f *right)
{
	CHECK_VECTOR2F(right);
	
	return Vector_init_Vector (v, right);
}

int Vector2f_compare (Vector2f *v, Vector2f *right)
{
	CHECK_VECTOR2F(v);
	CHECK_VECTOR2F(right);
	
	return Vector_compare (v, right);
}


int Vector2f_set_all (Vector2f *v, float value)
{
	CHECK_VECTOR2F(v);
	
	return Vector_init_two_elements (v, value, value);
}


int Vector2f_getX (Vector2f *v, float *result)
{
	CHECK_VECTOR2F(v);
	
	return Vector_get_data (v, result, 0);
}

int Vector2f_getY (Vector2f *v, float *result)
{
	CHECK_VECTOR2F(v);
	
	return Vector_get_data (v, result, 1);
}

int Vector2f_setX (Vector2f *v, float value)
{
	CHECK_VECTOR2F(v);
	
	return Vector_set_data (v, value, 0);
}

int Vector2f_setY (Vector2f *v, float value)
{
	CHECK_VECTOR2F(v);
	
	return Vector_set_data (v, value, 1);
}


int Vector2f_add_float (Vector2f *v, float value)
{
	CHECK_VECTOR2F(v);
	
	return Vector_add_float (v, value);
}

int Vector2f_sub_float (Vector2f *v, float value)
{
	CHECK_VECTOR2F(v);
	
	return Vector_sub_float (v, value);
}


int Vector2f_mul_float (Vector2f *v, float value)
{
	CHECK_VECTOR2F(v);
	
	return Vector_mul_float (v, value);
}


int Vector2f_div_float (Vector2f *v, float value)
{
	CHECK_VECTOR2F(v);
	
	return Vector_div_float (v, value);
}


int Vector2f_add_Vector2f (Vector2f *v, Vector2f *right)
{
	CHECK_VECTOR2F(v);
	CHECK_VECTOR2F(right);
	
	return Vector_add_Vector (v, right);
}

int Vector2f_sub_Vector2f (Vector2f *v, Vector2f *right)
{
	CHECK_VECTOR2F(v);
	CHECK_VECTOR2F(right);
	
	return Vector_sub_Vector (v, right);
}

int Vector2f_mul_Vector2f (Vector2f *v, float *result, Vector2f *right)
{
	return Vector2f_dot (v, result, right);
}

int Vector2f_emul_Vector2f (Vector3f *v, Vector3f *result, Vector3f *right)
{
	CHECK_VECTOR2F(v);
	CHECK_VECTOR2F(right);

	return Vector_emul_Vector (v, result, right, VECTOR2F_ROWS);
}

int Vector2f_ediv_Vector2f (Vector3f *v, Vector3f *result, Vector3f *right)
{
	CHECK_VECTOR2F(v);
	CHECK_VECTOR2F(right);

	return Vector_ediv_Vector (v, result, right, VECTOR2F_ROWS);
}

int Vector2f_cross_Vector2f (Vector2f *v, float *result, Vector2f *right)
{
	float v0, v1, right0, right1;
	
	CHECK_VECTOR2F(v);
	CHECK_VECTOR2F(right);
	
	MATHLIB_ASSERT (Vector2f_getX (v, &v0));
	MATHLIB_ASSERT (Vector2f_getY (v, &v1));
	MATHLIB_ASSERT (Vector2f_getX (right, &right0));
	MATHLIB_ASSERT (Vector2f_getY (right, &right1));
	*result = (v0 * right1 - v1 * right0);
	
	return 0;
}


int Vector2f_change_sign (Vector2f *v)
{
	CHECK_VECTOR2F(v);
	
	return Vector_change_sign (v);
}

int Vector2f_dot (Vector2f *v, float *result, Vector2f *right)
{
	CHECK_VECTOR2F(v);
	CHECK_VECTOR2F(right);

	return Vector_dot (v, result, right);
}

int Vector2f_length (Vector2f *v, float *result)
{
	CHECK_VECTOR2F(v);

	return Vector_length (v, result);
}

int Vector2f_norm (Vector2f *v, float *result)
{
	CHECK_VECTOR2F(v);

	return Vector_length (v, result);
}

int Vector2f_normalize (Vector2f *v)
{
	CHECK_VECTOR2F(v);

	return Vector_normalize (v);
}

int Vector2f_unit (Vector2f *v)
{
	CHECK_VECTOR2F(v);

	return Vector_normalize (v);
}
