/**
 * @file Vector.h
 *
 * math Vector
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "Vector.h"


// constructor
inline int Vector_init_zero (Vector *v, uint32_t n_rows)
{
	if (!v ||
		n_rows <= 0 || n_rows > VECTOR_MAX_ROWS)
		return -1;
	
	memset(v, 0, sizeof(Vector));
	v->rows = n_rows;

	return 0;
}

inline int Vector_init_default (Vector *v, uint32_t n_rows)
{
	return Vector_init_zero (v, n_rows);
}

inline int Vector_init_float_pointer (Vector *v, uint32_t n_rows, float *data)
{
	uint32_t i;

	if (!v || !data ||
		n_rows <= 0 || n_rows > VECTOR_MAX_ROWS)
		return -1;
	
	v->rows = n_rows;
	for (i = 0; i < n_rows; i++)
		v->data[i] = data[i];

	return 0;
}

// copy constructor
inline int Vector_init_Vector (Vector *v, Vector *right)
{
	//CHECK_VECTOR(right);
	if (!v)
		return -1;

	memcpy(v, right, sizeof(Vector));

	return 0;
}


inline int Vector_init_two_elements (Vector *v, float a, float b)
{
	MATHLIB_ASSERT (Vector_init_zero (v, 2));
	
	v->data[0] = a;
	v->data[1] = b;

	return 0;
}

inline int Vector_init_three_elements (Vector *v, float a, float b, float c)
{
	MATHLIB_ASSERT (Vector_init_zero (v, 3));

	v->data[0] = a;
	v->data[1] = b;
	v->data[2] = c;

	return 0;
}

inline int Vector_init_four_elements (Vector *v, float a, float b, float c, float d)
{
	MATHLIB_ASSERT (Vector_init_zero (v, 4));

	v->data[0] = a;
	v->data[1] = b;
	v->data[2] = c;
	v->data[3] = d;

	return 0;
}


inline int Vector_get_data (Vector *v, float *result, uint32_t row)
{
	//CHECK_VECTOR(v);
	CHECK_VECTOR_INDEX_PARAM (row, v->rows);
	if (!result)
		return -1;
	
	*result = v->data[row];
	
	return 0;
}

inline int Vector_set_data (Vector *v, float data, uint32_t row)
{
	//CHECK_VECTOR(v);
	CHECK_VECTOR_INDEX_PARAM (row, v->rows);
	
	v->data[row] = data;
	
	return 0;
}

inline int Vector_compare (Vector *v, Vector *right)
{
	uint32_t i;
	
	//CHECK_VECTOR(v);
	//CHECK_VECTOR(right);
	
	if (v->rows != right->rows)
		return 0;
	
	for (i = 0; i < v->rows; i++)
		if (fabsf((v->data[i] - right->data[i])) > 1e-30f)
			return 0;

	return 1;
}


inline int Vector_add_float (Vector *v, float value)
{
	uint32_t i;
	
	//CHECK_VECTOR(v);

	for (i = 0; i < v->rows; i++)
		v->data[i] += value;

	return 0;
}

inline int Vector_sub_float (Vector *v, float value)
{
	uint32_t i;
	
	//CHECK_VECTOR(v);

	for (i = 0; i < v->rows; i++)
		v->data[i] -= value;

	return 0;
}

inline int Vector_mul_float (Vector *v, float value)
{
	uint32_t i;
	
	//CHECK_VECTOR(v);

	for (i = 0; i < v->rows; i++)
		v->data[i] *= value;

	return 0;
}

inline int Vector_div_float (Vector *v, float value)
{
	uint32_t i;
	
	//CHECK_VECTOR(v);

	for (i = 0; i < v->rows; i++)
		v->data[i] /= value;

	return 0;
}

inline int Vector_add_Vector (Vector *v, Vector *right)
{
	uint32_t i;

	//CHECK_VECTOR(v);
	//CHECK_VECTOR(right);

	if (v->rows != right->rows)
		return -1;

	for (i = 0; i < v->rows; i++)
		v->data[i] += right->data[i];

	return 0;
}

inline int Vector_sub_Vector (Vector *v, Vector *right)
{
	uint32_t i;

	//CHECK_VECTOR(v);
	//CHECK_VECTOR(right);
	
	if (v->rows != right->rows)
		return -1;

	for (i = 0; i < v->rows; i++)
		v->data[i] -= right->data[i];

	return 0;
}


inline int Vector_change_sign (Vector *v)
{
	uint32_t i;

	//CHECK_VECTOR(v);

	for (i = 0; i < v->rows; i++)
		v->data[i] = -v->data[i];

	return 0;
}


inline int Vector_dot (Vector *v, float *result, Vector *right)
{
	uint32_t i;
	float result_value = 0;
	
	//CHECK_VECTOR(v);
	//CHECK_VECTOR(right);

	if (v->rows != right->rows)
		return -1;

	for (i = 0; i < v->rows; i++)
		result_value += v->data[i] * right->data[i];

	*result = result_value;
	
	return 0;
}

inline int Vector_length (Vector *v, float *result)
{
	float result_value = 0;
	
	MATHLIB_ASSERT (Vector_dot (v, &result_value, v));
	*result = sqrtf (result_value);
	
	return 0;
}

inline int Vector_normalize (Vector *v)
{
	float result_value = 0;
	
	MATHLIB_ASSERT (Vector_length (v, &result_value));
	
	return Vector_div_float (v, result_value);
}
