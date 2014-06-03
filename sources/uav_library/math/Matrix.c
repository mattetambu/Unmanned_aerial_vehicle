/**
 * @file Matrix.h
 *
 * math Matrix
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "Matrix.h"



// constructor
inline int Matrix_init_zero (Matrix *m, uint32_t n_rows, uint32_t n_cols)
{
	if (!m ||
		n_rows <= 0 || n_rows > MATRIX_MAX_ROWS ||
		n_cols <= 0 || n_cols > MATRIX_MAX_COLS)
		return -1;
	
	memset(m, 0, sizeof(Matrix));
	m->rows = n_rows;
	m->cols = n_cols;

	return 0;
}

inline int Matrix_init_default (Matrix *m, uint32_t n_rows, uint32_t n_cols)
{
	return Matrix_init_zero (m, n_rows, n_cols);
}	

inline int Matrix_init_identity (Matrix *m, uint32_t size)
{
	uint32_t i;
	
	MATHLIB_ASSERT (Matrix_init_zero (m, size, size));
	
	for (i = 0; i < size; i++)
		m->data[i][i] = 1.0f;
	
	return 0;
}

inline int Matrix_init_float_pointer (Matrix *m, uint32_t n_rows, uint32_t n_cols, float *data)
{
	uint32_t i, j;

	if (!m || !data ||
		n_rows <= 0 || n_rows > MATRIX_MAX_ROWS ||
		n_cols <= 0 || n_cols > MATRIX_MAX_COLS)
		return -1;
	
	m->rows = n_rows;
	m->cols = n_cols;

	for (i = 0; i < n_rows; i++)
		for (j = 0; j < n_cols; j++)
			m->data[i][j] = data[i*n_cols + j];

	return 0;
}

// copy constructor
inline int Matrix_init_Matrix (Matrix *m, Matrix *right)
{
	CHECK_MATRIX(right);
	if (!m)
		return -1;

	memcpy(m, right, sizeof(Matrix));

	return 0;
}



inline int Matrix_get_data (Matrix *m, float *result, uint32_t row, uint32_t col)
{
	CHECK_MATRIX(m);
	CHECK_MATRIX_INDEX_PARAM (row, m->rows);
	CHECK_MATRIX_INDEX_PARAM (col, m->cols);
	if (!result)
		return -1;
	
	*result = m->data[row][col];
	
	return 0;
}

inline int Matrix_set_data (Matrix *m, float data, uint32_t row, uint32_t col)
{
	CHECK_MATRIX(m);
	CHECK_MATRIX_INDEX_PARAM (row, m->rows);
	CHECK_MATRIX_INDEX_PARAM (col, m->cols);
	
	m->data[row][col] = data;
	
	return 0;
}

inline int Matrix_set_all (Matrix *m, float data)
{
	uint32_t i, j;
	
	CHECK_MATRIX(m);
	
	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			m->data[i][j] = data;
	
	return 0;
}

inline int Matrix_compare (Matrix *m, Matrix *right)
{
	uint32_t i, j;
	
	CHECK_MATRIX(m);
	CHECK_MATRIX(right);
	
	if (m->rows != right->rows ||
		m->cols != right->cols)
		return 0;
	
	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			if (fabsf(m->data[i][j] - right->data[i][j]) > 1e-30f)
				return 0;
	
	return 1;
}


inline int Matrix_add_float (Matrix *m, float value)
{
	uint32_t i, j;
	
	CHECK_MATRIX(m);

	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			m->data[i][j] += value;

	return 0;
}

inline int Matrix_sub_float (Matrix *m, float value)
{
	uint32_t i, j;
	
	CHECK_MATRIX(m);

	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			m->data[i][j] -= value;

	return 0;
}

inline int Matrix_mul_float (Matrix *m, float value)
{
	uint32_t i, j;
	
	CHECK_MATRIX(m);

	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			m->data[i][j] *= value;
			
	return 0;
}

inline int Matrix_div_float (Matrix *m, float value)
{
	uint32_t i, j;
	
	CHECK_MATRIX(m);

	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			m->data[i][j] /= value;

	return 0;
}

inline int Matrix_mul_Vector (Matrix *m, Vector *result, Vector *right)
{
	uint32_t i, j;
	float f_result;
	Vector T;
	
	CHECK_MATRIX(m);
	CHECK_VECTOR(right);
	
	if (m->cols != right->rows)
		return -1;
		
	MATHLIB_ASSERT (Vector_init_zero (&T, m->rows));
		
	for (i = 0; i < m->rows; i++) {
		f_result = 0;
		
		for (j = 0; j < m->cols; j++)
			f_result += m->data[i][j] * right->data[j];
		
		MATHLIB_ASSERT (Vector_set_data (&T, f_result, i));
	}
	
	return Vector_init_Vector (result, &T);
}

inline int Matrix_add_Matrix (Matrix *m, Matrix *right)
{
	uint32_t i, j;
	
	CHECK_MATRIX(m);
	CHECK_MATRIX(right);
	
	if (m->rows != right->rows ||
		m->cols != right->cols)
		return -1;

	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			m->data[i][j] += right->data[i][j];

	return 0;
}

inline int Matrix_sub_Matrix (Matrix *m, Matrix *right)
{
	uint32_t i, j;
	
	CHECK_MATRIX(m);
	CHECK_MATRIX(right);
	
	if (m->rows != right->rows ||
		m->cols != right->cols)
		return -1;

	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			m->data[i][j] -= right->data[i][j];

	return 0;
}

inline int Matrix_mul_Matrix (Matrix *m, Matrix *result, Matrix *right)
{
	uint32_t i, j, k;
	Matrix T;
	
	CHECK_MATRIX(m);
	CHECK_MATRIX(right);
	
	if (m->cols != right->rows)
		return -1;
	
	MATHLIB_ASSERT (Matrix_init_zero (&T, m->rows, right->cols));

	for (i = 0; i < m->rows; i++)
		for (j = 0; j < right->cols; j++)
			for (k = 0; k < right->rows; k++)
				T.data[i][j] += m->data[i][k] * right->data[k][j];

	return Matrix_init_Matrix (result, &T);
}

inline int Matrix_div_Matrix (Matrix *m, Matrix *right)
{
	Matrix T, Inv;
	
	CHECK_MATRIX(m);
	CHECK_MATRIX(right);
	
	if (m->cols != right->cols ||
		right->rows != right->cols)
		return -1;

	MATHLIB_ASSERT (Matrix_init_Matrix (&Inv, right));
	MATHLIB_ASSERT (Matrix_inverse (&Inv));
	MATHLIB_ASSERT (Matrix_mul_Matrix (m, &T, &Inv));

	return Matrix_init_Matrix (m, &T);
}

inline int Matrix_change_sign (Matrix *m)
{
	uint32_t i, j;

	CHECK_MATRIX(m);

	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			m->data[i][j] = -m->data[i][j];

	return 0;
}

inline int Matrix_transpose (Matrix *m, Matrix *result)
{
	uint32_t i, j;
	Matrix T;
	
	CHECK_MATRIX(m);
	Matrix_init_zero (&T, m->cols, m->rows);
	
	for (i = 0; i < m->rows; i++)
		for (j = 0; j < m->cols; j++)
			T.data[j][i] = m->data[i][j];

	return Matrix_init_Matrix (result, &T);
}

inline int Matrix_swap_rows (Matrix *m, uint32_t a, uint32_t b)
{
	uint32_t j;
	float tmp;
	
	if (a == b)
		return 0;
	
	CHECK_MATRIX(m);
	CHECK_MATRIX_INDEX_PARAM (a, m->rows);
	CHECK_MATRIX_INDEX_PARAM (b, m->rows);
	
	for (j = 0; j < m->cols; j++)
	{
		tmp = m->data[a][j];
		m->data[a][j] = m->data[b][j];
		m->data[b][j] = tmp;
	}
	
	return 0;
}

inline int Matrix_swap_cols (Matrix *m, uint32_t a, uint32_t b)
{
	uint32_t i;
	float tmp;
	
	if (a == b)
		return 0;
	
	CHECK_MATRIX(m);
	CHECK_MATRIX_INDEX_PARAM (a, m->rows);
	CHECK_MATRIX_INDEX_PARAM (b, m->rows);
	
	for (i = 0; i < m->rows; i++)
	{
		tmp = m->data[i][a];
		m->data[i][a] = m->data[i][b];
		m->data[i][b] = tmp;
	}
	
	return 0;
}

inline int Matrix_inverse (Matrix *m)
{
	CHECK_MATRIX(m);
	if (m->rows != m->cols)
		return -1;
	
	uint32_t c, i, j, k, n;
	uint32_t N = m->rows;
	float U_nn = 0, U_in = 0, U_ii = 0, U_ij = 0, U_nk = 0, U_ik = 0;
	float L_in = 0, L_ij = 0, Y_ic = 0, Y_jc = 0, X_ic = 0, X_jc = 0;
	
	Matrix U, L, P, X, Y;
	MATHLIB_ASSERT (Matrix_init_Matrix (&U, m));
	MATHLIB_ASSERT (Matrix_init_identity (&L, N));
	MATHLIB_ASSERT (Matrix_init_identity (&P, N));
	
	// for all diagonal elements
	for (n = 0; n < N; n++) {
		// if diagonal is zero, swap with row below
		MATHLIB_ASSERT (Matrix_get_data (&U, &U_nn, n, n));
		if (fabsf(U_nn) < 1e-8f) {
			//printf("trying pivot for row %d\n",n);
			for (i = 0; i < N; i++) {
				if (i == n) continue;

				//printf("\ttrying row %d\n",i);
				MATHLIB_ASSERT (Matrix_get_data (&U, &U_in, i, n));
				if (fabsf(U_in) > 1e-8f) {
					//printf("swapped %d\n",i);
					MATHLIB_ASSERT (Matrix_swap_rows (&U, i, n));
					MATHLIB_ASSERT (Matrix_swap_rows (&P, i, n));
				}
			}
		}
		
		//printf("A:\n"); A.print();
		//printf("U:\n"); U.print();
		//printf("P:\n"); P.print();
		//fflush(stdout);
		
		// failsafe
		MATHLIB_ASSERT (Matrix_get_data (&U, &U_nn, n, n));
		if (fabsf(U_nn) < 1e-8f) {
			return -1;
		}

		// for all rows below diagonal
		for (i = (n + 1); i < N; i++) {
			MATHLIB_ASSERT (Matrix_get_data (&U, &U_in, i, n));
			MATHLIB_ASSERT (Matrix_get_data (&U, &U_nn, n, n));
			MATHLIB_ASSERT (Matrix_set_data (&L, U_in/U_nn, i, n));

			// add i-th row and n-th row
			// multiplied by: -a(i,n)/a(n,n)
			for (k = n; k < N; k++) {
				MATHLIB_ASSERT (Matrix_get_data (&L, &L_in, i, n));
				MATHLIB_ASSERT (Matrix_get_data (&U, &U_nk, n, k));
				MATHLIB_ASSERT (Matrix_get_data (&U, &U_ik, i, k));
				MATHLIB_ASSERT (Matrix_set_data (&U, U_ik-(L_in*U_nk), i, k));
			}
		}
	}
	
	//printf("L:\n"); L.print();
	//printf("U:\n"); U.print();
	//fflush(stdout);
	
	
	// solve LY=P*I for Y by forward subst
	MATHLIB_ASSERT (Matrix_init_Matrix (&Y, &P));

	// for all columns of Y
	for (c = 0; c < N; c++) {
		// for all rows of L
		for (i = 0; i < N; i++) {
			// for all columns of L
			for (j = 0; j < i; j++) {
				// for all existing y
				// subtract the component they
				// contribute to the solution
				MATHLIB_ASSERT (Matrix_get_data (&L, &L_ij, i, j));
				MATHLIB_ASSERT (Matrix_get_data (&Y, &Y_jc, j, c));
				MATHLIB_ASSERT (Matrix_get_data (&Y, &Y_ic, i, c));
				MATHLIB_ASSERT (Matrix_set_data (&Y, Y_ic-(L_ij*Y_jc), i, c));
			}

			// divide by the factor
			// on current
			// term to be solved
			// Y(i,c) /= L(i,i);
			// but L(i,i) = 1.0
		}
	}

	//printf("Y:\n"); Y.print();
	//fflush(stdout);
	
	// solve Ux=y for x by back subst
	MATHLIB_ASSERT (Matrix_init_Matrix (&X, &Y));

	// for all columns of X
	for (c = 0; c < N; c++) {
		// for all rows of U
		for (k = 0; k < N; k++) {
			// have to go in reverse order
			i = N - 1 - k;

			// for all columns of U
			for (j = i + 1; j < N; j++) {
				// for all existing x
				// subtract the component they
				// contribute to the solution
				MATHLIB_ASSERT (Matrix_get_data (&U, &U_ij, i, j));
				MATHLIB_ASSERT (Matrix_get_data (&X, &X_jc, j, c));
				MATHLIB_ASSERT (Matrix_get_data (&X, &X_ic, i, c));
				MATHLIB_ASSERT (Matrix_set_data (&X, X_ic-(U_ij*X_jc), i, c));
			}

			// divide by the factor
			// on current
			// term to be solved
			MATHLIB_ASSERT (Matrix_get_data (&U, &U_ii, i, i));
			MATHLIB_ASSERT (Matrix_get_data (&X, &X_ic, i, c));
			MATHLIB_ASSERT (Matrix_set_data (&X, X_ic/U_ii, i, c));
		}
	}
			
	//printf("X:\n"); X.print();
	//fflush(stdout);
	
	return Matrix_init_Matrix (m, &X);
}
