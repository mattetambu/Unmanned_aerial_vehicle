/**
 * @file Matrix.h
 *
 * math Matrix
 */


#ifndef	_MATH_MATRIX_H_
#define	_MATH_MATRIX_H_

	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>

	#include "Vector.h"

	#define MATRIX_MAX_ROWS				4
	#define MATRIX_MAX_COLS				4
	#define CHECK_MATRIX(m)				if (!m || m->rows <= 0 || m->rows > MATRIX_MAX_ROWS || m->cols <= 0 || m->cols > MATRIX_MAX_COLS) return -1;
	#define CHECK_MATRIX_INDEX_PARAM(i, max)	if (i < 0 || i >= max) return -1;

	typedef struct Matrix
	{
		uint32_t rows;
		uint32_t cols;
		float data[MATRIX_MAX_ROWS][MATRIX_MAX_COLS];
	} Matrix;

	
	typedef struct Matrix Dcm;


	// constructor
	inline int Matrix_init_zero (Matrix *m, uint32_t n_rows, uint32_t n_cols);
	inline int Matrix_init_default (Matrix *m, uint32_t n_rows, uint32_t n_cols);
	inline int Matrix_init_identity (Matrix *m, uint32_t size);
	inline int Matrix_init_float_pointer (Matrix *m, uint32_t n_rows, uint32_t n_cols, float *data);
	
	// copy constructor
	inline int Matrix_init_Matrix (Matrix *m, Matrix *right);	

	inline int Matrix_get_data (Matrix *m, float *result, uint32_t row, uint32_t col);
	inline int Matrix_set_data (Matrix *m, float data, uint32_t row, uint32_t col);
	inline int Matrix_set_all (Matrix *m, float data);
	inline int Matrix_compare (Matrix *m, Matrix *right);
	
	inline int Matrix_add_float (Matrix *m, float value);
	inline int Matrix_sub_float (Matrix *m, float value);
	inline int Matrix_mul_float (Matrix *m, float value);
	inline int Matrix_div_float (Matrix *m, float value);
	inline int Matrix_mul_Vector (Matrix *m, Vector *result, Vector *right);
	inline int Matrix_add_Matrix (Matrix *m, Matrix *right);
	inline int Matrix_sub_Matrix (Matrix *m, Matrix *right);
	inline int Matrix_mul_Matrix (Matrix *m, Matrix *result, Matrix *right);
	inline int Matrix_div_Matrix (Matrix *m, Matrix *right);
	
	inline int Matrix_change_sign (Matrix *m);
	inline int Matrix_transpose (Matrix *m, Matrix *result);
	inline int Matrix_swap_rows (Matrix *m, uint32_t a, uint32_t b);
	inline int Matrix_swap_cols (Matrix *m, uint32_t a, uint32_t b);
	inline int Matrix_inverse (Matrix *m);
	
#endif
