/**
 * @file EulerAngles_test.h
 *
 * math EulerAngles
 */

#include "Test_common.h"
#include "../Dcm.h"
#include "../EulerAngles.h"
#include "../Quaternion.h"
#include "../Vector3f.h"


int EulerAngles_test ()
{
	EulerAngles e, e2;
	Vector3f v;
	Dcm d;
	Quaternion q;
	float Phi, Theta, Psi;
	
	fprintf (stderr, "Test EulerAngles\t: ");
	
	// test float ctor
	MATHLIB_ASSERT (Vector3f_init_components (&v, 0.1f, 0.2f, 0.3f));
	MATHLIB_ASSERT (EulerAngles_init_components (&e, 0.1f, 0.2f, 0.3f));
	
	MATHLIB_ASSERT (EulerAngles_getPhi (&e, &Phi));
	MATHLIB_ASSERT (EulerAngles_getTheta (&e, &Theta));
	MATHLIB_ASSERT (EulerAngles_getPsi (&e, &Psi));
	
	MATHLIB_ASSERT (VectorEqual(&e, &v));
	MATHLIB_ASSERT (equal(Phi, 0.1f));
	MATHLIB_ASSERT (equal(Theta, 0.2f));
	MATHLIB_ASSERT (equal(Psi, 0.3f));

	// test quat ctor
	MATHLIB_ASSERT (Quaternion_init_EulerAngles (&q, &e));
	MATHLIB_ASSERT (EulerAngles_init_Quaternion (&e2, &q));
	
	MATHLIB_ASSERT (VectorEqual(&e2, &v));

	// test dcm ctor
	MATHLIB_ASSERT (Dcm_init_EulerAngles (&d, &e));
	MATHLIB_ASSERT (EulerAngles_init_Dcm (&e2, &d));
	
	MATHLIB_ASSERT (VectorEqual(&e2, &v));

	// test accessors
	MATHLIB_ASSERT (Vector3f_init_components (&v, 0.4f, 0.5f, 0.6f));
	MATHLIB_ASSERT (EulerAngles_setPhi (&e, 0.4f));
	MATHLIB_ASSERT (EulerAngles_setTheta (&e, 0.5f));
	MATHLIB_ASSERT (EulerAngles_setPsi (&e, 0.6f));
	
	MATHLIB_ASSERT (VectorEqual(&e, &v));
	
	fprintf (stderr, "PASS\n");
	
	return 0;
}
