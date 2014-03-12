/**
 * @file Matrix_test.h
 *
 * Matrix code
 */

#include "Test_common.h"
#include "../Matrix.h"


static float data_testA[] = {
	1, 2, 3,
	4, 5, 6
};

static float data_testB[] = {
	0, 1, 3,
	7, -1, 2
};

static float data_testC[] = {
	0, 1,
	2, 1,
	3, 2
};

static float data_testD[] = {
	0, 1, 2,
	2, 1, 4,
	5, 2, 0
};

static float data_testE[] = {
	1, -1, 2,
	0, 2, 3,
	2, -1, 1
};

static float data_testF[] = {
	3.777e006f, 2.915e007f, 0.000e000f,
	2.938e007f, 2.267e008f, 0.000e000f,
	0.000e000f, 0.000e000f, 6.033e008f
};

static Matrix testA, testB, testC, testD, testE, testF;


int MatrixAdd_test ()
{
	Matrix testAdd1, testAdd2;
	float data_test[] = {
		1.0f, 3.0f, 6.0f,
		11.0f, 4.0f, 8.0f
	};
	
	fprintf (stderr, "Test Matrix Add\t\t: ");
	
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testAdd1, 2, 3, data_test));
	MATHLIB_ASSERT (Matrix_init_Matrix (&testAdd2, &testA));
	MATHLIB_ASSERT (Matrix_add_Matrix (&testAdd2, &testB));
	
	MATHLIB_ASSERT (MatrixEqual (&testAdd1, &testAdd2));
	
	fprintf (stderr, "PASS\n");
	
	return 0;
}

int MatrixSub_test ()
{
	Matrix testSub1, testSub2;
	float data_test[] = {
		1.0f, 1.0f, 0.0f,
		-3.0f, 6.0f, 4.0f
	};
	
	fprintf (stderr, "Test Matrix Sub\t\t: ");
	
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testSub1, 2, 3, data_test));
	MATHLIB_ASSERT (Matrix_init_Matrix (&testSub2, &testA));
	MATHLIB_ASSERT (Matrix_sub_Matrix (&testSub2, &testB));
	
	MATHLIB_ASSERT (MatrixEqual (&testSub1, &testSub2));
	
	fprintf (stderr, "PASS\n");
	
	return 0;
}

int MatrixMult_test ()
{
	Matrix testMul1, testMul2;
	float data_test[] = {
		7.0f, -1.0f,  2.0f,
		7.0f,  1.0f,  8.0f,
		14.0f,  1.0f, 13.0f
	};
	
	fprintf (stderr, "Test Matrix Mult\t: ");

	MATHLIB_ASSERT (Matrix_init_float_pointer (&testMul1, 3, 3, data_test));
	MATHLIB_ASSERT (Matrix_mul_Matrix (&testC, &testMul2, &testB));
	
	MATHLIB_ASSERT (MatrixEqual (&testMul1, &testMul2));
	
	fprintf (stderr, "PASS\n");
	
	return 0;
}

int MatrixInv_test ()
{
	Matrix testInv1, testInv2, testInv3;
	float data_test[] = {
		-0.0012518f,  0.0001610f, 0.0000000f,
		0.0001622f, -0.0000209f, 0.0000000f,
		0.0000000f,  0.0000000f, 1.6580e-9f
	};
	
	fprintf (stderr, "Test Matrix Inv\t\t: ");
	
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testInv3, 3, 3, data_test));
	MATHLIB_ASSERT (Matrix_init_Matrix (&testInv1, &testF));
	MATHLIB_ASSERT (Matrix_init_Matrix (&testInv2, &testF));
	MATHLIB_ASSERT (Matrix_inverse (&testInv2));
	
	MATHLIB_ASSERT (MatrixEqual (&testInv3, &testInv2));
	MATHLIB_ASSERT (MatrixEqual (&testInv1, &testF));
	
	fprintf (stderr, "PASS\n");
	
	return 0;
}

int MatrixDiv_test ()
{
	Matrix testDiv1, testDiv2;
	float data_test[] = {
		0.2222222f, 0.5555556f, -0.1111111f,
		0.0f,       1.0f,         1.0,
		-4.1111111f, 1.2222222f,  4.5555556f
	};
	
	fprintf (stderr, "Test Matrix Div\t\t: ");
	
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testDiv1, 3, 3, data_test));
	MATHLIB_ASSERT (Matrix_init_Matrix (&testDiv2, &testD));
	MATHLIB_ASSERT (Matrix_div_Matrix (&testDiv2, &testE));
	
	MATHLIB_ASSERT (MatrixEqual (&testDiv1, &testDiv2));
	
	fprintf (stderr, "PASS\n");
	
	return 0;
}


int Matrix_test ()
{
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testA, 2, 3, data_testA));
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testB, 2, 3, data_testB));
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testC, 3, 2, data_testC));
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testD, 3, 3, data_testD));
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testE, 3, 3, data_testE));
	MATHLIB_ASSERT (Matrix_init_float_pointer (&testF, 3, 3, data_testF));
	
	MatrixAdd_test ();
	MatrixSub_test ();
	MatrixMult_test ();
	MatrixInv_test ();
	MatrixDiv_test ();
	return 0;
}
