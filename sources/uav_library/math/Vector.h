/**
 * @file Vector.h
 *
 * math Vector
 */


#ifndef	_MATH_VECTOR_H_
#define	_MATH_VECTOR_H_

	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>
	#include <string.h>

	#ifdef DEBUG
		#define MATHLIB_ASSERT(test)		if ((test) != 0) {	\
												fprintf (stderr, "WARNING: Assert failed (file:%s, row:%d )\n", __FILE__, __LINE__);	\
												return -1;	\
											}
	#else
		#define MATHLIB_ASSERT(test)		test;
	#endif

	#define VECTOR_MAX_ROWS				4
	#define CHECK_VECTOR(v)				if (!v || v->rows <= 0 || v->rows > VECTOR_MAX_ROWS) return -1;
	#define CHECK_VECTOR_INDEX_PARAM(i, max)	if (i < 0 || i >= max) return -1;
	
	typedef struct Vector
	{
		uint32_t rows;
		float data[VECTOR_MAX_ROWS];
	} Vector;

	typedef struct Vector EulerAngles;
	typedef struct Vector Quaternion;
	typedef struct Vector Vector2f;
	typedef struct Vector Vector3f;


	// constructor
	inline int Vector_init_zero (Vector *v, uint32_t n_rows);
	inline int Vector_init_default (Vector *v, uint32_t n_rows);
	inline int Vector_init_float_pointer (Vector *v, uint32_t n_rows, float *data);
	
	// copy constructor
	inline int Vector_init_Vector (Vector *v, Vector *right);
	
	inline int Vector_init_two_elements (Vector *v, float a, float b);
	inline int Vector_init_three_elements (Vector *v, float a, float b, float c);
	inline int Vector_init_four_elements (Vector *v, float a, float b, float c, float d);	
	inline int Vector_get_data (Vector *v, float *result, uint32_t row);
	inline int Vector_set_data (Vector *v, float data, uint32_t row);
	
	inline int Vector_compare (Vector *v, Vector *right);	
	inline int Vector_add_float (Vector *v, float value);
	inline int Vector_sub_float (Vector *v, float value);
	inline int Vector_mul_float (Vector *v, float value);
	inline int Vector_div_float (Vector *v, float value);
	inline int Vector_add_Vector (Vector *v, Vector *right);
	inline int Vector_sub_Vector (Vector *v, Vector *right);
	inline int Vector_emul_Vector (Vector *v, Vector *result, Vector *right, uint32_t n_rows);
	inline int Vector_ediv_Vector (Vector *v, Vector *result, Vector *right, uint32_t n_rows);

	inline int Vector_change_sign (Vector *v);
	inline int Vector_dot (Vector *v, float *result, Vector *right);
	inline int Vector_length (Vector *v, float *result);
	inline int Vector_normalize (Vector *v);	
	
#endif
