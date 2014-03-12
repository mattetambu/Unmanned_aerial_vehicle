/**
 * @file Vector_test.h
 *
 * math Vector
 */

#include "Test_common.h"
#include "../Vector.h"


static float data_v_testA[] = {1, 3};
static float data_v_testB[] = {4, 1};

static Vector v_testA, v_testB;



int VectorAdd_test ()
{
	Vector testAdd1, testAdd2;
	float data_test[] = {5.0f, 4.0f};
	
	fprintf (stderr, "Test Vector Add\t\t: ");
	
	MATHLIB_ASSERT (Vector_init_float_pointer (&testAdd1, 2, data_test));
	MATHLIB_ASSERT (Vector_init_Vector (&testAdd2, &v_testA));
	MATHLIB_ASSERT (Vector_add_Vector (&testAdd2, &v_testB));
	
	MATHLIB_ASSERT (VectorEqual (&testAdd1, &testAdd2));
	
	fprintf (stderr, "PASS\n");
	
	return 0;
}

int VectorSub_test ()
{
	Vector testSub1, testSub2;
	float data_test[] = {-3.0f, 2.0f};
	
	fprintf (stderr, "Test Vector Sub\t\t: ");
	
	MATHLIB_ASSERT (Vector_init_float_pointer (&testSub1, 2, data_test));
	MATHLIB_ASSERT (Vector_init_Vector (&testSub2, &v_testA));
	MATHLIB_ASSERT (Vector_sub_Vector (&testSub2, &v_testB));
	
	MATHLIB_ASSERT (VectorEqual (&testSub1, &testSub2));
	
	fprintf (stderr, "PASS\n");
	
	return 0;
}

int Vector_test ()
{
	MATHLIB_ASSERT (Vector_init_float_pointer (&v_testA, 2, data_v_testA));
	MATHLIB_ASSERT (Vector_init_float_pointer (&v_testB, 2, data_v_testB));
		
	VectorAdd_test ();
	VectorSub_test ();

	return 0;
}
