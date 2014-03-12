/**
 * @file Dcm.h
 *
 * math Dcm
 */


#ifndef	_MATH_DCM_H_
#define	_MATH_DCM_H_

	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>

	#include "Vector.h"
	#include "Matrix.h"

	
	#define DCM_ROWS					3
	#define DCM_COLS					3
	#define CHECK_DCM(d)				if (!d || d->rows != DCM_ROWS || d->cols != DCM_COLS) return -1;


	// constructor
	int Dcm_init_zero (Dcm *d);
	int Dcm_init_identity (Dcm *d);
	int Dcm_init_default (Dcm *d);
	int Dcm_init_components (Dcm *d, float c00, float c01, float c02,
							float c10, float c11, float c12,
							float c20, float c21, float c22);
	int Dcm_init_float_Matrix (Dcm *d, float data[3][3]);
	int Dcm_init_float_pointer (Dcm *d, float *data);
	int Dcm_init_Quaternion (Dcm *d, Quaternion *q);
	int Dcm_init_EulerAngles (Dcm *d, EulerAngles *e);

	// copy constructor
	int Dcm_init_Dcm (Dcm *d, Dcm *right);	

	int Dcm_get_data (Dcm *d, float *result, uint32_t row, uint32_t col);
	int Dcm_set_data (Dcm *d, float data, uint32_t row, uint32_t col);
	int Dcm_set_all (Dcm *d, float data);
	int Dcm_compare (Dcm *d, Dcm *right);
	
	int Dcm_add_float (Dcm *d, float value);
	int Dcm_sub_float (Dcm *d, float value);
	int Dcm_mul_float (Dcm *d, float value);
	int Dcm_div_float (Dcm *d, float value);
	int Dcm_mul_Vector (Dcm *d, Vector *result, Vector *right);
	int Dcm_add_Dcm (Dcm *d, Dcm *right);
	int Dcm_sub_Dcm (Dcm *d, Dcm *right);
	int Dcm_mul_Dcm (Dcm *d, Dcm *result, Dcm *right);
	int Dcm_div_Dcm (Dcm *d, Dcm *right);
	
	int Dcm_change_sign (Dcm *d);
	int Dcm_transpose (Dcm *d, Dcm *result);
	int Dcm_swap_rows (Dcm *d, uint32_t a, uint32_t b);
	int Dcm_swap_cols (Dcm *d, uint32_t a, uint32_t b);
	int Dcm_inverse (Dcm *d);
	
#endif
