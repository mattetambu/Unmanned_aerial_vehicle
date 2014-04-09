/*
 * @file inertial_filter.h
 *
 */


#ifndef	_INERTIAL_FILTER_H_
#define	_INERTIAL_FILTER_H_

	void inertial_filter_predict(float dt, float x[3]);
	void inertial_filter_correct(float e, float dt, float x[3], int i, float w);

#endif
