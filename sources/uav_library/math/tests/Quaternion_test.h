/**
 * @file Quaternion_test.h
 *
 * math Vector
 */

#include "Test_common.h"
#include "../Quaternion.h"
#include "../Dcm.h"
#include "../EulerAngles.h"


int Quaternion_test ()
{
	Quaternion q, q2;
	EulerAngles e;
	Dcm d;
	float qA, qB, qC, qD;
	
	fprintf (stderr, "Test Quaternion\t\t: ");
	
	// test default ctor
	MATHLIB_ASSERT (Quaternion_init_default (&q));
	MATHLIB_ASSERT (Quaternion_getA (&q, &qA));
	MATHLIB_ASSERT (Quaternion_getB (&q, &qB));
	MATHLIB_ASSERT (Quaternion_getC (&q, &qC));
	MATHLIB_ASSERT (Quaternion_getD (&q, &qD));
	
	MATHLIB_ASSERT (equal(qA, 1.0f));
	MATHLIB_ASSERT (equal(qB, 0.0f));
	MATHLIB_ASSERT (equal(qC, 0.0f));
	MATHLIB_ASSERT (equal(qD, 0.0f));
	
	// test float ctor
	MATHLIB_ASSERT (Quaternion_init_components (&q, 0.1825742f, 0.3651484f, 0.5477226f, 0.7302967f));
	MATHLIB_ASSERT (Quaternion_getA (&q, &qA));
	MATHLIB_ASSERT (Quaternion_getB (&q, &qB));
	MATHLIB_ASSERT (Quaternion_getC (&q, &qC));
	MATHLIB_ASSERT (Quaternion_getD (&q, &qD));
	
	MATHLIB_ASSERT (equal(qA, 0.1825742f));
	MATHLIB_ASSERT (equal(qB, 0.3651484f));
	MATHLIB_ASSERT (equal(qC, 0.5477226f));
	MATHLIB_ASSERT (equal(qD, 0.7302967f));
	
	// test euler ctor
	MATHLIB_ASSERT (EulerAngles_init_components (&e, 0.1f, 0.2f, 0.3f));
	MATHLIB_ASSERT (Quaternion_init_components (&q, 0.983347f, 0.034271f, 0.106021f, 0.143572f));
	MATHLIB_ASSERT (Quaternion_init_EulerAngles (&q2, &e));
	
	MATHLIB_ASSERT (VectorEqual(&q, &q2));
	
	// test dcm ctor
	MATHLIB_ASSERT (Dcm_init_default (&d));
	MATHLIB_ASSERT (Quaternion_init_Dcm (&q, &d));
	MATHLIB_ASSERT (Quaternion_init_default (&q2));
	
	MATHLIB_ASSERT (VectorEqual(&q, &q2));
	
	// test accessors
	MATHLIB_ASSERT (Quaternion_setA (&q, 0.1f));
	MATHLIB_ASSERT (Quaternion_setB (&q, 0.2f));
	MATHLIB_ASSERT (Quaternion_setC (&q, 0.3f));
	MATHLIB_ASSERT (Quaternion_setD (&q, 0.4f));
	MATHLIB_ASSERT (Quaternion_init_components (&q2, 0.1f, 0.2f, 0.3f, 0.4f));
	
	MATHLIB_ASSERT (VectorEqual(&q, &q2));
	fprintf (stderr, "PASS\n");
	
	return 0;
}
