/**
 * @file Quaternion.h
 *
 * math Quaternion
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Quaternion.h"
#include "EulerAngles.h"
#include "Dcm.h"


// constructor
int Quaternion_init_zero (Quaternion *q)
{
	return Vector_init_zero (q, QUATERNION_ROWS);
}

int Quaternion_init_default (Quaternion *q)
{
	MATHLIB_ASSERT (Vector_init_zero (q, QUATERNION_ROWS));
		
	return Quaternion_setA (q, 1.0f);
}

int Quaternion_init_float_pointer (Quaternion *q, float *data)
{
	return Vector_init_float_pointer (q, QUATERNION_ROWS, data);
}

int Quaternion_init_components (Quaternion *q, float a, float b, float c, float d)
{
	return Vector_init_four_elements (q, a, b, c, d);
}

/*
int Quaternion_init_Vector (Quaternion *q, Vector *v)
{
	CHECK_QUATERNION(v);
	
	return Vector_init_Vector (q, v);
}
*/

int Quaternion_init_EulerAngles (Quaternion *q, EulerAngles *e)
{
	float Phi, Theta, Psi;
	double cosPhi_2, cosTheta_2, cosPsi_2, sinPhi_2, sinTheta_2, sinPsi_2;
	
	CHECK_EULERANGLES(e);
	if (!q)
		return -1;
	
	MATHLIB_ASSERT (EulerAngles_getPhi (e, &Phi));
	MATHLIB_ASSERT (EulerAngles_getTheta (e, &Theta));
	MATHLIB_ASSERT (EulerAngles_getPsi (e, &Psi));
	
	cosPhi_2 = cos (Phi / 2.0);
	cosTheta_2 = cos (Theta / 2.0);
	cosPsi_2 = cos (Psi / 2.0);
	sinPhi_2 = sin (Phi / 2.0);
	sinTheta_2 = sin (Theta / 2.0);
	sinPsi_2 = sin (Psi / 2.0);
	
	q->rows = QUATERNION_ROWS;
	MATHLIB_ASSERT (Quaternion_setA (q, cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2));
	MATHLIB_ASSERT (Quaternion_setB (q, sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2));
	MATHLIB_ASSERT (Quaternion_setC (q, cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2));
	MATHLIB_ASSERT (Quaternion_setD (q, cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2));
		
	return 0;
}

int Quaternion_init_Dcm (Quaternion *q, Dcm *d)
{
	float d00, d11, d22;
	
	CHECK_DCM(d);
	if (!q)
		return -1;

	MATHLIB_ASSERT (Dcm_get_data (d, &d00, 0, 0));
	MATHLIB_ASSERT (Dcm_get_data (d, &d11, 1, 1));
	MATHLIB_ASSERT (Dcm_get_data (d, &d22, 2, 2));
	
	q->rows = QUATERNION_ROWS;
	MATHLIB_ASSERT (Quaternion_setA (q, 0.5 * sqrt (1.0 + (d00 + d11 + d22))));
	MATHLIB_ASSERT (Quaternion_setB (q, 0.5 * sqrt (1.0 + (d00 - d11 - d22))));
	MATHLIB_ASSERT (Quaternion_setC (q, 0.5 * sqrt (1.0 + (-d00 + d11 - d22))));
	MATHLIB_ASSERT (Quaternion_setD (q, 0.5 * sqrt (1.0 + (-d00 - d11 + d22))));
	
	return 0;
}

// copy constructor
int Quaternion_init_Quaternion (Quaternion *q, Quaternion *right)
{
	CHECK_QUATERNION(right);
	
	return Vector_init_Vector (q, right);
}

int Quaternion_compare (Quaternion *q, Quaternion *right)
{
	CHECK_QUATERNION(q);
	CHECK_QUATERNION(right);
	
	return Vector_compare (q, right);
}


int Quaternion_set_all (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_init_four_elements (q, value, value, value, value);
}


int Quaternion_getA (Quaternion *q, float *result)
{
	CHECK_QUATERNION(q);

	return Vector_get_data (q, result, 0);
}

int Quaternion_getB (Quaternion *q, float *result)
{
	CHECK_QUATERNION(q);

	return Vector_get_data (q, result, 1);
}

int Quaternion_getC (Quaternion *q, float *result)
{
	CHECK_QUATERNION(q);

	return Vector_get_data (q, result, 2);
}

int Quaternion_getD (Quaternion *q, float *result)
{
	CHECK_QUATERNION(q);

	return Vector_get_data (q, result, 3);
}

int Quaternion_setA (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_set_data (q, value, 0);
}

int Quaternion_setB (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_set_data (q, value, 1);
}

int Quaternion_setC (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_set_data (q, value, 2);
}

int Quaternion_setD (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_set_data (q, value, 3);
}


int Quaternion_add_float (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_add_float (q, value);
}

int Quaternion_sub_float (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_sub_float (q, value);
}


int Quaternion_mul_float (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_mul_float (q, value);
}


int Quaternion_div_float (Quaternion *q, float value)
{
	CHECK_QUATERNION(q);
	
	return Vector_div_float (q, value);
}


int Quaternion_add_Quaternion (Quaternion *q, Quaternion *right)
{
	CHECK_QUATERNION(q);
	CHECK_QUATERNION(right);
	
	return Vector_add_Vector (q, right);
}

int Quaternion_sub_Quaternion (Quaternion *q, Quaternion *right)
{
	CHECK_QUATERNION(q);
	CHECK_QUATERNION(right);
	
	return Vector_sub_Vector (q, right);
}


int Quaternion_change_sign (Quaternion *q)
{
	CHECK_QUATERNION(q);
	
	return Vector_change_sign (q);
}

int Quaternion_dot (Quaternion *q, float *result, Quaternion *right)
{
	CHECK_QUATERNION(q);
	CHECK_QUATERNION(right);

	return Vector_dot (q, result, right);
}

int Quaternion_length (Quaternion *q, float *result)
{
	CHECK_QUATERNION(q);

	return Vector_length (q, result);
}

int Quaternion_norm (Quaternion *q, float *result)
{
	CHECK_QUATERNION(q);

	return Vector_length (q, result);
}

int Quaternion_normalize (Quaternion *q)
{
	CHECK_QUATERNION(q);

	return Vector_normalize (q);
}

int Quaternion_unit (Quaternion *q)
{
	CHECK_QUATERNION(q);

	return Vector_normalize (q);
}


int Quaternion_derivative (Quaternion *q, Vector *v)
{
	float a, b, c, d;
	float x, y, z;
	
	CHECK_QUATERNION(q);
	if (!v || v->rows != 3)
		return -1;

	MATHLIB_ASSERT (Quaternion_getA (q, &a));
	MATHLIB_ASSERT (Quaternion_getB (q, &b));
	MATHLIB_ASSERT (Quaternion_getC (q, &c));
	MATHLIB_ASSERT (Quaternion_getD (q, &d));
	
	MATHLIB_ASSERT (Vector_get_data (v, &x, 0));
	MATHLIB_ASSERT (Vector_get_data (v, &y, 1));
	MATHLIB_ASSERT (Vector_get_data (v, &z, 2));
	
	float dataQ[] = {
		a, -b, -c, -d,
		b,  a, -d,  c,
		c,  d,  a, -b,
		d, -c,  b,  a
	};
	
	Quaternion qq;
	MATHLIB_ASSERT (Quaternion_init_components (&qq, 0.0f, x, y, z));
	MATHLIB_ASSERT (Quaternion_mul_float (&qq, 0.5f));
	
	Matrix Q;
	MATHLIB_ASSERT (Matrix_init_float_pointer (&Q, 4, 4, dataQ));
	MATHLIB_ASSERT (Matrix_mul_Vector (&Q, q, &qq));
	
	return 0;
}
