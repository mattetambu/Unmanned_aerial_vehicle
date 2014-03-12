/**
 * @file Vector3f_test.h
 *
 * math Vector
 */

#include "Test_common.h"
#include "../Vector3f.h"




int Vector3f_Mul_test ()
{
	Vector3f testMul1, testMul2;
	float data_v3_testA[] = {1, 2, 3};
	float data_v3_testB[] = {2, 4, 6};

	fprintf (stderr, "Test Vector3f Mul\t: ");

	MATHLIB_ASSERT (Vector3f_init_float_pointer (&testMul1, data_v3_testA));
	MATHLIB_ASSERT (Vector3f_init_float_pointer (&testMul2, data_v3_testB));
	MATHLIB_ASSERT (Vector3f_mul_float (&testMul1, 2.0f));

	MATHLIB_ASSERT (VectorEqual (&testMul1, &testMul2));

	fprintf (stderr, "PASS\n");

	return 0;
}


int Vector3f_Init_test ()
{
	Vector3f v3f_1, v3f_2;
	float v0, v1, v2;
	
	fprintf (stderr, "Test Vector3f Init\t: ");
	
	// test float ctor
	MATHLIB_ASSERT (Vector3f_init_components (&v3f_1, 1, 2, 3));
	MATHLIB_ASSERT (Vector3f_getX (&v3f_1, &v0));
	MATHLIB_ASSERT (Vector3f_getY (&v3f_1, &v1));
	MATHLIB_ASSERT (Vector3f_getZ (&v3f_1, &v2));
	
	MATHLIB_ASSERT (equal(v0, 1));
	MATHLIB_ASSERT (equal(v1, 2));
	MATHLIB_ASSERT (equal(v2, 3));

	MATHLIB_ASSERT (Vector3f_init_Vector3f (&v3f_2, &v3f_1));
	MATHLIB_ASSERT (VectorEqual (&v3f_1, &v3f_2));


	fprintf (stderr, "PASS\n");

	return 0;
}

int Vector3f_test ()
{
	Vector3f_Init_test ();
	Vector3f_Mul_test ();
	
	return 0;
}

