// mixer_main.h

#ifndef	_MIXER_MAIN_H_
#define	_MIXER_MAIN_H_

	#include "../../uav_library/common.h"
	#include "../../uav_type.h"


	/**
	 * Precalculated rotor mix.
	 */
	typedef struct rotor_scale {
		float roll_scale;	/**< scales roll for this rotor */
		float pitch_scale;	/**< scales pitch for this rotor */
		float yaw_scale;	/**< scales yaw for this rotor */
	} rotor_scale;


	/** simple channel scaler */
	typedef struct mixer_scaler_s {
		float negative_scale;
		float positive_scale;
		float offset;
		float min_output;
		float max_output;
	} mixer_scaler_s;

	/** mixer input */
	typedef struct mixer_control_s {
		uint8_t control_group;	/**< group from which the input reads */
		uint8_t control_index;	/**< index within the control group */
		struct mixer_scaler_s scaler;		/**< scaling applied to the input before use */
	} mixer_control_s;



	typedef struct multirotor_mixer_t {
		float roll_scale;
		float pitch_scale;
		float yaw_scale;
		float deadband;

		unsigned rotor_count;
		const rotor_scale *rotors;
	} multirotor_mixer_t;


	typedef struct fixedwing_mixer_t {
		uint8_t control_count;	/**< number of inputs */

		struct mixer_control_s controls[4];
	} fixedwing_mixer_t;



	/* function prototypes */
	void* mixer_thread_main (void* args);
	
	/* global variables */
	// empty
	
#endif
