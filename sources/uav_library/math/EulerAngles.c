/**
 * @file EulerAngles.h
 *
 * math EulerAngles
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "EulerAngles.h"
#include "Quaternion.h"
#include "Dcm.h"


// constructor
int EulerAngles_init_zero (EulerAngles *e)
{
	return Vector_init_zero (e, EULERANGLES_ROWS);
}

int EulerAngles_init_default (EulerAngles *e)
{
	return Vector_init_zero (e, EULERANGLES_ROWS);
}

int EulerAngles_init_float_pointer (EulerAngles *e, float *data)
{
	return Vector_init_float_pointer (e, EULERANGLES_ROWS, data);
}

int EulerAngles_init_components (EulerAngles *e, float x, float y, float z)
{
	return Vector_init_three_elements (e, x, y, z);
}

int EulerAngles_init_Dcm (EulerAngles *e, Dcm *d)
{
	float Phi, Theta;
	float d00, d01, d02, d10, d11, d12, d20, d21, d22;
	
	CHECK_DCM(d);
	if (!e)
		return -1;

	MATHLIB_ASSERT (Dcm_get_data (d, &d00, 0, 0));
	MATHLIB_ASSERT (Dcm_get_data (d, &d01, 0, 1));
	MATHLIB_ASSERT (Dcm_get_data (d, &d02, 0, 2));
	MATHLIB_ASSERT (Dcm_get_data (d, &d10, 1, 0));
	MATHLIB_ASSERT (Dcm_get_data (d, &d11, 1, 1));
	MATHLIB_ASSERT (Dcm_get_data (d, &d12, 1, 2));
	MATHLIB_ASSERT (Dcm_get_data (d, &d20, 2, 0));
	MATHLIB_ASSERT (Dcm_get_data (d, &d21, 2, 1));
	MATHLIB_ASSERT (Dcm_get_data (d, &d22, 2, 2));
		
	e->rows = EULERANGLES_ROWS;
	MATHLIB_ASSERT (EulerAngles_setTheta (e, asinf(-d20)));
	MATHLIB_ASSERT (EulerAngles_getTheta (e, &Theta));

	if (fabsf(Theta - M_PI_2) < 1.0e-3f) {
		MATHLIB_ASSERT (EulerAngles_setPhi (e, 0.0f));
		MATHLIB_ASSERT (EulerAngles_getPhi (e, &Phi));
		MATHLIB_ASSERT (EulerAngles_setPsi (e, atan2f(d12 - d01, d02 + d11) + Phi));

	} else if (fabsf(Theta + M_PI_2) < 1.0e-3f) {
		MATHLIB_ASSERT (EulerAngles_setPhi (e, 0.0f));
		MATHLIB_ASSERT (EulerAngles_getPhi (e, &Phi));
		MATHLIB_ASSERT (EulerAngles_setPsi (e, atan2f(d12 - d01, d02 + d11) - Phi));

	} else {
		MATHLIB_ASSERT (EulerAngles_setPhi (e, atan2f(d21, d22)));
		MATHLIB_ASSERT (EulerAngles_setPsi (e, atan2f(d10, d00)));
	}
	
	return 0;
}

int EulerAngles_init_Quaternion (EulerAngles *e, Quaternion *q)
{
	CHECK_QUATERNION(q);
	if (!e)
		return -1;
		
	Dcm d;
	MATHLIB_ASSERT (Dcm_init_Quaternion (&d, q));
	
	return EulerAngles_init_Dcm (e, &d);
}

// copy constructor
int EulerAngles_init_EulerAngles (EulerAngles *e, EulerAngles *right)
{
	CHECK_EULERANGLES(right);
	
	return Vector_init_Vector (e, right);
}

int EulerAngles_compare (EulerAngles *e, EulerAngles *right)
{
	CHECK_EULERANGLES(e);
	CHECK_EULERANGLES(right);
	
	return Vector_compare (e, right);
}


int EulerAngles_set_all (EulerAngles *e, float value)
{
	CHECK_EULERANGLES(e);
	
	return Vector_init_three_elements (e, value, value, value);
}


int EulerAngles_getPhi (EulerAngles *e, float *result)
{
	CHECK_EULERANGLES(e);

	return Vector_get_data (e, result, 0);
}

int EulerAngles_getTheta (EulerAngles *e, float *result)
{
	CHECK_EULERANGLES(e);

	return Vector_get_data (e, result, 1);
}

int EulerAngles_getPsi (EulerAngles *e, float *result)
{
	CHECK_EULERANGLES(e);

	return Vector_get_data (e, result, 2);
}

int EulerAngles_setPhi (EulerAngles *e, float value)
{
	CHECK_EULERANGLES(e);
	
	return Vector_set_data (e, value, 0);
}

int EulerAngles_setTheta (EulerAngles *e, float value)
{
	CHECK_EULERANGLES(e);
	
	return Vector_set_data (e, value, 1);
}

int EulerAngles_setPsi (EulerAngles *e, float value)
{
	CHECK_EULERANGLES(e);
	
	return Vector_set_data (e, value, 2);
}


int EulerAngles_add_float (EulerAngles *e, float value)
{
	CHECK_EULERANGLES(e);
	
	return Vector_add_float (e, value);
}

int EulerAngles_sub_float (EulerAngles *e, float value)
{
	CHECK_EULERANGLES(e);
	
	return Vector_sub_float (e, value);
}


int EulerAngles_mul_float (EulerAngles *e, float value)
{
	CHECK_EULERANGLES(e);
	
	return Vector_mul_float (e, value);
}


int EulerAngles_div_float (EulerAngles *e, float value)
{
	CHECK_EULERANGLES(e);
	
	return Vector_div_float (e, value);
}


int EulerAngles_add_EulerAngles (EulerAngles *e, EulerAngles *right)
{
	CHECK_EULERANGLES(e);
	CHECK_EULERANGLES(right);
	
	return Vector_add_Vector (e, right);
}

int EulerAngles_sub_EulerAngles (EulerAngles *e, EulerAngles *right)
{
	CHECK_EULERANGLES(e);
	CHECK_EULERANGLES(right);
	
	return Vector_sub_Vector (e, right);
}


int EulerAngles_change_sign (EulerAngles *e)
{
	CHECK_EULERANGLES(e);
	
	return Vector_change_sign (e);
}

int EulerAngles_dot (EulerAngles *e, float *result, EulerAngles *right)
{
	CHECK_EULERANGLES(e);
	CHECK_EULERANGLES(right);

	return Vector_dot (e, result, right);
}

int EulerAngles_length (EulerAngles *e, float *result)
{
	CHECK_EULERANGLES(e);

	return Vector_length (e, result);
}

int EulerAngles_norm (EulerAngles *e, float *result)
{
	CHECK_EULERANGLES(e);

	return Vector_length (e, result);
}

int EulerAngles_normalize (EulerAngles *e)
{
	CHECK_EULERANGLES(e);

	return Vector_normalize (e);
}

int EulerAngles_unit (EulerAngles *e)
{
	CHECK_EULERANGLES(e);

	return Vector_normalize (e);
}