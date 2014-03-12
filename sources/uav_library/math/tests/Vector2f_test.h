/**
 * @file Vector2f_test.h
 *
 * math Vector
 */

#include "Test_common.h"
#include "../Vector2f.h"





int Vector2f_Mul_test ()
{
	Vector2f testMul1, testMul2;
	float data_v2_testA[] = {1, 3};
	float data_v2_testB[] = {4, 1};
	float data_test = 7, result;

	fprintf (stderr, "Test Vector2f Mul\t: ");

	MATHLIB_ASSERT (Vector2f_init_float_pointer (&testMul1, data_v2_testA));
	MATHLIB_ASSERT (Vector2f_init_float_pointer (&testMul2, data_v2_testB));
	MATHLIB_ASSERT (Vector2f_mul_Vector2f (&testMul1, &result, &testMul2));

	MATHLIB_ASSERT (equal (data_test, result));

	fprintf (stderr, "PASS\n");

	return 0;
}


int Vector2f_Init_test ()
{
	Vector2f v2f_1, v2f_2;
	float v0, v1;
	
	fprintf (stderr, "Test Vector2f Init\t: ");
	
	// test float ctor
	MATHLIB_ASSERT (Vector2f_init_components (&v2f_1, 1, 2));
	MATHLIB_ASSERT (Vector2f_getX (&v2f_1, &v0));
	MATHLIB_ASSERT (Vector2f_getY (&v2f_1, &v1));
	
	MATHLIB_ASSERT (equal(v0, 1));
	MATHLIB_ASSERT (equal(v1, 2));

	MATHLIB_ASSERT (Vector2f_init_Vector2f (&v2f_2, &v2f_1));
	MATHLIB_ASSERT (VectorEqual (&v2f_1, &v2f_2));

	fprintf (stderr, "PASS\n");

	return 0;
}

int Vector2f_test ()
{
	Vector2f_Init_test ();
	Vector2f_Mul_test ();
	
	return 0;
}
