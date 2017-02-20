/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* @file kite.h
*
* @author Andreas Okholm   <bapstroman@gmail.com
*
*/

#ifndef KITE_H
#define KITE_H

#include "vtol_type.h"
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>

class Kite : public VtolType
{

public:
	Kite(VtolAttitudeControl *_att_controller);
	~Kite();

	// Overwrite some of the Vtol_type methods
	virtual void update_vtol_state();
	virtual void update_transition_state();
	virtual void update_mc_state();
	virtual void update_fw_state();
	virtual void fill_actuator_outputs();
	virtual void waiting_on_tecs();

private:

	// example
	struct {
		float airspeed_trans;
		float trans_forward_roll;
		float trans_forward_thrust;
		float trans_forward_pitch;
		float trans_forward_duration_max;
		float trans_backwards_pitch;
		float trans_backwards_roll;
		float trans_backwards_thrust;
		float wind_speed;
		math::Vector<3> pos_b;

	} _params_kite;

	struct {
		param_t airspeed_trans;
		param_t trans_forward_roll;
		param_t trans_forward_thrust;
		param_t trans_forward_pitch;
		param_t trans_forward_duration_max;
		param_t trans_backwards_pitch;
		param_t trans_backwards_roll;
		param_t trans_backwards_thrust;
		param_t wind_speed;
		param_t x_pos_b;
		param_t y_pos_b;
		param_t z_pos_b;

	} _params_handles_kite;

	// vtol mode specific for the kite
	enum vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT,	/**< vtol is in front transition part 1 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	struct {
		vtol_mode flight_mode;			/**< vtol flight mode, defined by enum vtol_mode */
		hrt_abstime transition_start;	/**< absoulte time at which front transition started */
	} _vtol_schedule;

	// other local variables
	// float _airspeed_tot; 		/** speed estimation for propwash controlled surfaces */


	float _yaw_transition_start;  // roll angle at the start of transition (kite)
	float _roll_transition_start;  // roll angle at the start of transition (kite)
	float _pitch_transition_start;  // pitch angle at the start of transition (kite)
	float _thrust_transition_start; // throttle value when we start the front transition

	float _speed;
	float _airspeed_ratio;

	float _transition_ratio;

	bool _fail_safe_hover;


	/**
	 * set initial values before transitioning.
	 */
	void set_transition_starting_values(bool forward);

	/**
	* calculate the elevator correction
	*/
	float elevator_correction();

	/**
	 * Update parameters.
	 */
	void parameters_update(); // deleted virtual

	/**
	 * Update transition ratio based on time passed since start.
	 */
	void update_transition_ratio();

	/**
	* update external VTOL state
	*/
	void update_external_VTOL_state();

};
#endif
