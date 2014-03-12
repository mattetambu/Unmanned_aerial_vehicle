/**
 * @file Dcm.h
 *
 * math Dcm
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Dcm.h"
#include "Quaternion.h"
#include "EulerAngles.h"

// constructor
int Dcm_init_zero (Dcm *d)
{
	return Matrix_init_zero (d, DCM_ROWS, DCM_COLS);
}

int Dcm_init_identity (Dcm *d)
{
	return Matrix_init_identity (d, DCM_ROWS);
}

int Dcm_init_default (Dcm *d)
{
	return Matrix_init_identity (d, DCM_ROWS);
}

int Dcm_init_components (Dcm *d, float c00, float c01, float c02,
						float c10, float c11, float c12,
						float c20, float c21, float c22)
{
	if (!d)
		return -1;
	
	d->rows = DCM_ROWS;
	d->cols = DCM_COLS;
	
	MATHLIB_ASSERT (Matrix_set_data (d, c00, 0, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, c01, 0, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, c02, 0, 2));
	MATHLIB_ASSERT (Matrix_set_data (d, c10, 1, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, c11, 1, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, c12, 1, 2));
	MATHLIB_ASSERT (Matrix_set_data (d, c20, 2, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, c21, 2, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, c22, 2, 2));
		
	return 0;
}

int Dcm_init_float_Matrix (Dcm *d, float data[3][3])
{
	//return Matrix_init_float_pointer (d, DCM_ROWS, DCM_COLS, &data[0][0]);
	uint32_t i, j;
	
	if (!d || !data)
		return -1;
	
	d->rows = DCM_ROWS;
	d->cols = DCM_COLS;
	
	for (i = 0; i < DCM_ROWS; i++) {
		for (j = 0; j < DCM_COLS; j++) {
			MATHLIB_ASSERT (Matrix_set_data (d, data[i][j], i, j));
		}
	}
	
	return 0;
}

int Dcm_init_float_pointer (Dcm *d, float *data)
{
	return Matrix_init_float_pointer (d, DCM_ROWS, DCM_COLS, data);
}

int Dcm_init_Quaternion (Dcm *d, Quaternion *q)
{
	float A, B, C, D;
	double aSq, bSq, cSq, dSq;
	
	CHECK_QUATERNION(q);
	if (!d)
		return -1;
			
	MATHLIB_ASSERT (Quaternion_getA (q, &A));
	MATHLIB_ASSERT (Quaternion_getB (q, &B));
	MATHLIB_ASSERT (Quaternion_getC (q, &C));
	MATHLIB_ASSERT (Quaternion_getD (q, &D));
		
	aSq = A * A;
	bSq = B * B;
	cSq = C * C;
	dSq = D * D;
	
	d->rows = DCM_ROWS;
	d->cols = DCM_COLS;
	MATHLIB_ASSERT (Matrix_set_data (d, aSq + bSq - cSq - dSq, 0, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, 2.0 * (B * C - A * D), 0, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, 2.0 * (B * D + A * C), 0, 2));
	MATHLIB_ASSERT (Matrix_set_data (d, 2.0 * (B * C + A * D), 1, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, aSq - bSq + cSq - dSq, 1, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, 2.0 * (C * D - A * B), 1, 2));
	MATHLIB_ASSERT (Matrix_set_data (d, 2.0 * (B * D - A * C), 2, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, 2.0 * (C * D + A * B), 2, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, aSq - bSq - cSq + dSq, 2, 2));
		
	return 0;
}

int Dcm_init_EulerAngles (Dcm *d, EulerAngles *e)
{
	float Phi, Theta, Psi;
	double cosPhi, cosTheta, cosPsi, sinPhi, sinTheta, sinPsi;
	
	CHECK_EULERANGLES(e);
	if (!d)
		return -1;
			
	MATHLIB_ASSERT (EulerAngles_getPhi (e, &Phi));
	MATHLIB_ASSERT (EulerAngles_getTheta (e, &Theta));
	MATHLIB_ASSERT (EulerAngles_getPsi (e, &Psi));
	
	cosPhi = cos (Phi);
	cosTheta = cos (Theta);
	cosPsi = cos (Psi);
	sinPhi = sin (Phi);
	sinTheta = sin (Theta);
	sinPsi = sin (Psi);

	d->rows = DCM_ROWS;
	d->cols = DCM_COLS;
	MATHLIB_ASSERT (Matrix_set_data (d, cosTheta * cosPsi, 0, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, -cosPhi * sinPsi + sinPhi * sinTheta * cosPsi, 0, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, sinPhi * sinPsi + cosPhi * sinTheta * cosPsi, 0, 2));
	MATHLIB_ASSERT (Matrix_set_data (d, cosTheta * sinPsi, 1, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, cosPhi * cosPsi + sinPhi * sinTheta * sinPsi, 1, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, -sinPhi * cosPsi + cosPhi * sinTheta * sinPsi, 1, 2));
	MATHLIB_ASSERT (Matrix_set_data (d, -sinTheta, 2, 0));
	MATHLIB_ASSERT (Matrix_set_data (d, sinPhi * cosTheta, 2, 1));
	MATHLIB_ASSERT (Matrix_set_data (d, cosPhi * cosTheta, 2, 2));
		
	return 0;
}


// copy constructor
int Dcm_init_Dcm (Dcm *d, Dcm *right)
{
	CHECK_DCM(right);
	
	return Matrix_init_Matrix (d, right);
}



int Dcm_get_data (Dcm *d, float *result, uint32_t row, uint32_t col)
{
	CHECK_DCM(d);
	
	return Matrix_get_data (d, result, row, col);
}

int Dcm_set_data (Dcm *d, float data, uint32_t row, uint32_t col)
{
	CHECK_DCM(d);
	
	return Matrix_set_data (d, data, row, col);
}

int Dcm_set_all (Dcm *d, float data)
{
	CHECK_DCM(d);
	
	return Matrix_set_all (d, data);
}

int Dcm_compare (Dcm *d, Dcm *right)
{
	CHECK_DCM(d);
	CHECK_DCM(right);
	
	return Matrix_compare (d, right);
}


int Dcm_add_float (Dcm *d, float value)
{
	CHECK_DCM(d);
	
	return Matrix_add_float (d, value);
}

int Dcm_sub_float (Dcm *d, float value)
{
	CHECK_DCM(d);
	
	return Matrix_sub_float (d, value);
}

int Dcm_mul_float (Dcm *d, float value)
{
	CHECK_DCM(d);
	
	return Matrix_mul_float (d, value);
}

int Dcm_div_float (Dcm *d, float value)
{
	CHECK_DCM(d);
	
	return Matrix_div_float (d, value);
}

int Dcm_mul_Vector (Dcm *d, Vector *result, Vector *right)
{
	CHECK_DCM(d);
	
	return Matrix_mul_Vector (d, result, right);
}

int Dcm_add_Dcm (Dcm *d, Dcm *right)
{
	CHECK_DCM(d);
	CHECK_DCM(right);
	
	return Matrix_add_Matrix (d, right);
}

int Dcm_sub_Dcm (Dcm *d, Dcm *right)
{
	CHECK_DCM(d);
	CHECK_DCM(right);
	
	return Matrix_sub_Matrix (d, right);
}

int Dcm_mul_Dcm (Dcm *d, Dcm *result, Dcm *right)
{
	CHECK_DCM(d);
	CHECK_DCM(right);
	
	return Matrix_mul_Matrix (d, result, right);
}

int Dcm_div_Dcm (Dcm *d, Dcm *right)
{
	CHECK_DCM(d);
	CHECK_DCM(right);
	
	return Matrix_div_Matrix (d, right);
}

int Dcm_change_sign (Dcm *d)
{
	CHECK_DCM(d);
	
	return Matrix_change_sign (d);
}

int Dcm_transpose (Dcm *d, Dcm *result)
{
	CHECK_DCM(d);
	
	return Matrix_transpose (d, result);
}

int Dcm_swap_rows (Dcm *d, uint32_t a, uint32_t b)
{
	CHECK_DCM(d);
	
	return Matrix_swap_rows (d, a, b);
}

int Dcm_swap_cols (Dcm *d, uint32_t a, uint32_t b)
{
	CHECK_DCM(d);
	
	return Matrix_swap_cols (d, a, b);
}

int Dcm_inverse (Dcm *d)
{
	CHECK_DCM(d);
	
	return Matrix_inverse (d);
}
