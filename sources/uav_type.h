// mixer_main.h

#ifndef	_MAV_TYPE_H_
#define	_MAV_TYPE_H_

	#include "uav_library/common.h"

	typedef enum rotor_geometry {
		QUAD_X = 0,	/**< quad in X configuration */
		QUAD_PLUS,	/**< quad in + configuration */
		QUAD_V,		/**< quad in V configuration */
		QUAD_WIDE,	/**< quad in wide configuration */
		HEX_X,		/**< hex in X configuration */
		HEX_PLUS,	/**< hex in + configuration */
		OCTA_X,
		OCTA_PLUS,
		OCTA_COX,

		MAX_GEOMETRY
	} rotor_geometry;
	
	typedef enum mav_type {
		FIXEDWING = 0,
		MULTIROTOR,
	} mav_type;
	
	/* function prototypes */
	// empty
	
	/* global variables */
	extern bool_t is_rotary_wing;
	extern rotor_geometry r_geometry;
	
#endif
