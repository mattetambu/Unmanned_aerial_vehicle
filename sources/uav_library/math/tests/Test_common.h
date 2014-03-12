/**
 * @file Test_common.cpp
 *
 * Test library code
 */


#ifndef	_MATH_TEXT_COMMON_H_
#define	_MATH_TEXT_COMMON_H_

	#include <stdio.h>
	#include <math.h>
	#include <stdlib.h>
	#include "../Vector.h"
	#include "../Matrix.h"


	int equal(float a, float b)
	{
		float epsilon = 1e-5;
		float diff = fabsf(a - b);

		if (diff > epsilon) {
			fprintf (stderr, "\n\tnot equal ->\ta: %12.8f\n\t\t\tb: %12.8f\n", a, b);
			return -1;
		}
	
		return 0;
	}

	int VectorEqual(Vector *a, Vector *b)
	{
		int i;

		if (a->rows != b->rows) {
			fprintf (stderr, "row number not equal a: %d, b:%d\n", a->rows, b->rows);
			return -1;
		}
	
		for (i = 0; i < a->rows; i++) {
			if (equal(a->data[i], b->data[i]) != 0) {
				fprintf (stderr, "\telement mismatch (%d)\n", i);
				return -1;
			}
		}

		return 0;
	}

	int MatrixEqual(Matrix *a, Matrix *b)
	{
		int i, j;

		if (a->rows != b->rows) {
			fprintf (stderr, "row number not equal a: %d, b:%d\n", a->rows, b->rows);
			return -1;

		}
		if (a->cols != b->cols) {
			fprintf (stderr, "column number not equal a: %d, b:%d\n", a->cols, b->cols);
			return -1;
		}

		for (i = 0; i < a->rows; i++)
			for (j = 0; j < a->cols; j++) {
				if (equal(a->data[i][j], b->data[i][j]) != 0) {
					fprintf (stderr, "\telement mismatch (%d, %d)\n", i, j);
					return -1;
				}
			}

		return 0;
	}


	int matrix_print (Matrix *m, uint32_t n_rows, uint32_t n_cols)
	{
		uint32_t i, j;

		fprintf (stderr, "\n\tmatrix:\n");
		for (i = 0; i < m->rows; i++) {
			for (j = 0; j < m->cols; j++) {
				fprintf (stderr, "\t%f", m->data[i][j]);
			}
			fprintf (stderr, "\n");
		}
		fprintf (stderr, "\n");

		return 0;
	}


#endif
