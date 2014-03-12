/**
 * @file limits.h
 *
 * Limiting / constrain helper functions
 */

#ifndef	_MATH_LIMITS_H_
#define	_MATH_LIMITS_H_

	#include <math.h>
	#include <stdint.h>

	/*
	float min_f (float val1, float val2)
	{
		return (val1 < val2) ? val1 : val2;
	}

	int min_i (int val1, int val2)
	{
		return (val1 < val2) ? val1 : val2;
	}

	unsigned min_u (unsigned val1, unsigned val2)
	{
		return (val1 < val2) ? val1 : val2;
	}

	uint64_t min_ul (uint64_t val1, uint64_t val2)
	{
		return (val1 < val2) ? val1 : val2;
	}

	double min_d (double val1, double val2)
	{
		return (val1 < val2) ? val1 : val2;
	}

	float max_f (float val1, float val2)
	{
		return (val1 > val2) ? val1 : val2;
	}

	int max_i (int val1, int val2)
	{
		return (val1 > val2) ? val1 : val2;
	}

	unsigned max_u (unsigned val1, unsigned val2)
	{
		return (val1 > val2) ? val1 : val2;
	}

	uint64_t max_ul (uint64_t val1, uint64_t val2)
	{
		return (val1 > val2) ? val1 : val2;
	}

	double max_d (double val1, double val2)
	{
		return (val1 > val2) ? val1 : val2;
	}


	float constrain_f (float val, float min, float max)
	{
		return (val < min) ? min : ((val > max) ? max : val);
	}

	int constrain_i (int val, int min, int max)
	{
		return (val < min) ? min : ((val > max) ? max : val);
	}

	unsigned constrain_u (unsigned val, unsigned min, unsigned max)
	{
		return (val < min) ? min : ((val > max) ? max : val);
	}

	uint64_t constrain_ul (uint64_t val, uint64_t min, uint64_t max)
	{
		return (val < min) ? min : ((val > max) ? max : val);
	}

	double constrain_d (double val, double min, double max)
	{
		return (val < min) ? min : ((val > max) ? max : val);
	}

	float deg_to_rad_f (float degrees)
	{
		return (degrees / 180.0f) * (float) M_PI;
	}

	double deg_to_rad_d (double degrees)
	{
		return (degrees / 180.0) * M_PI;
	}

	float rad_to_deg_f (float radians)
	{
		return (radians / (float) M_PI) * 180.0f;
	}

	double rad_to_deg_d (double radians)
	{
		return (radians / M_PI) * 180.0;
	}
	*/

	#define min(val1, val2)						(val1 < val2) ? val1 : val2
	#define max(val1, val2)						(val1 > val2) ? val1 : val2
	#define constrain(val, min, max)			(val < min) ? min : ((val > max) ? max : val)
	#define check_out_of_bounds(val, min, max)	(val < min) ? 1 : ((val > max) ? 1 : 0)
	#define check_finite(val)					(!isnan(val) && !isinf(val))
	#define radians(degrees)					(degrees / 180.0f) * (float) M_PI
	#define degrees(radians)					(radians / (float) M_PI) * 180.0f

#endif
