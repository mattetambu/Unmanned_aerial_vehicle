/**
 * @file Dcm_test.h
 *
 * math direction cosine Matrix
 */

#include "Test_common.h"
#include "../Dcm.h"
#include "../Quaternion.h"
#include "../EulerAngles.h"
#include "../Vector3f.h"


int Dcm_test ()
{
	Dcm d, d2;
	EulerAngles e;
	Quaternion q;
	Matrix m;
	Vector3f v, vB, vRes;
	
	
	fprintf (stderr, "Test Dcm\t\t: ");
	
	// test default ctor
	MATHLIB_ASSERT (Dcm_init_default (&d));
	MATHLIB_ASSERT (Matrix_init_identity (&m, d.rows));
	
	MATHLIB_ASSERT (MatrixEqual(&d, &m));
	
	// test quat ctor
	MATHLIB_ASSERT (Quaternion_init_components (&q, 0.983347f, 0.034271f, 0.106021f, 0.143572f));
	MATHLIB_ASSERT (Dcm_init_Quaternion (&d, &q));
	MATHLIB_ASSERT (Dcm_init_components (&d2, 0.9362934f, -0.2750958f,  0.2183507f,
										0.2896295f,  0.9564251f, -0.0369570f,
										-0.1986693f,  0.0978434f,  0.9751703f));
	
	MATHLIB_ASSERT (MatrixEqual(&d, &d2));
	
	// test euler ctor
	MATHLIB_ASSERT (EulerAngles_init_components (&e, 0.1f, 0.2f, 0.3f));
	MATHLIB_ASSERT (Dcm_init_EulerAngles (&d, &e));
	
	MATHLIB_ASSERT (MatrixEqual(&d, &d2));
	
	// rotations 1
	MATHLIB_ASSERT (Vector3f_init_components (&vB, 1, 2, 3));
	MATHLIB_ASSERT (Vector3f_init_components (&v, -2.0f, 1.0f, 3.0f));
	MATHLIB_ASSERT (EulerAngles_init_components (&e, 0.0f, 0.0f, M_PI_2));
	MATHLIB_ASSERT (Dcm_init_EulerAngles (&d, &e));
	MATHLIB_ASSERT (Dcm_mul_Vector (&d, &vRes, &vB));
	
	MATHLIB_ASSERT (VectorEqual(&v, &vRes));
	
	// rotations 2
	MATHLIB_ASSERT (Vector3f_init_components (&v, 3.0f, 2.0f, -1.0f));
	MATHLIB_ASSERT (EulerAngles_init_components (&e, 0.0f, M_PI_2, 0.0f));
	MATHLIB_ASSERT (Dcm_init_EulerAngles (&d, &e));
	MATHLIB_ASSERT (Dcm_mul_Vector (&d, &vRes, &vB));
	
	MATHLIB_ASSERT (VectorEqual(&v, &vRes));
	
	// rotations 3
	MATHLIB_ASSERT (Vector3f_init_components (&v, 1.0f, -3.0f, 2.0f));
	MATHLIB_ASSERT (EulerAngles_init_components (&e, M_PI_2, 0.0f, 0.0f));
	MATHLIB_ASSERT (Dcm_init_EulerAngles (&d, &e));
	MATHLIB_ASSERT (Dcm_mul_Vector (&d, &vRes, &vB));
	
	MATHLIB_ASSERT (VectorEqual(&v, &vRes));
	
	// rotations 4
	MATHLIB_ASSERT (Vector3f_init_components (&v, 3.0f, 2.0f, -1.0f));
	MATHLIB_ASSERT (EulerAngles_init_components (&e, M_PI_2, M_PI_2, M_PI_2));
	MATHLIB_ASSERT (Dcm_init_EulerAngles (&d, &e));
	MATHLIB_ASSERT (Dcm_mul_Vector (&d, &vRes, &vB));
	
	MATHLIB_ASSERT (VectorEqual(&v, &vRes));
		
	fprintf (stderr, "PASS\n");
	
	return 0;
}
