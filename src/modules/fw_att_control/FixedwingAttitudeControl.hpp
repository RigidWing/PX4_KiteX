/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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

#include <px4_module.h>
#include <drivers/drv_hrt.h>
#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/fw_turning.h> // KITEX
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h> // KITEX
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <vtol_att_control/vtol_type.h>

using matrix::Eulerf;
using matrix::Quatf;

using uORB::Subscription;

class FixedwingAttitudeControl final : public ModuleBase<FixedwingAttitudeControl>
{
public:
	FixedwingAttitudeControl();
	~FixedwingAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static FixedwingAttitudeControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	int		_att_sub{-1};				/**< vehicle attitude */
	int		_att_sp_sub{-1};			/**< vehicle attitude setpoint */
	int		_battery_status_sub{-1};		/**< battery status subscription */
	int		_global_pos_sub{-1};			/**< global position subscription */
	int		_local_pos_sub;				/**< vehicle local position */ // KITEX
	int		_manual_sub{-1};			/**< notification of manual control updates */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_vcontrol_mode_sub{-1};			/**< vehicle status subscription */
	int		_vehicle_land_detected_sub{-1};		/**< vehicle land detected subscription */
	int		_vehicle_status_sub{-1};		/**< vehicle status subscription */

	orb_advert_t	_rate_sp_pub{nullptr};			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub{nullptr};		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub{nullptr};		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub{nullptr};		/**< actuator control group 1 setpoint (Airframe) */
	orb_advert_t	_fw_turning_sp_pub; 			// KITEX
	orb_advert_t	_rate_ctrl_status_pub{nullptr};		/**< rate controller status publication */

	orb_id_t _rates_sp_id{nullptr};	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id{nullptr};	// pointer to correct actuator controls0 uORB metadata structure
	orb_id_t _attitude_setpoint_id{nullptr};
	orb_id_t _fw_turning_sp_id{nullptr};  // KITEX

	actuator_controls_s			_actuators {};		/**< actuator control inputs */
	actuator_controls_s			_actuators_airframe {};	/**< actuator control inputs */
	manual_control_setpoint_s		_manual {};		/**< r/c channel data */
	vehicle_attitude_s			_att {};		/**< vehicle attitude setpoint */
	vehicle_attitude_setpoint_s		_att_sp {};		/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};	/**< vehicle control mode */
	fw_turning_s 				_fw_turning_sp {}; 	// KITEX
	vehicle_global_position_s		_global_pos {};		/**< global position */
	vehicle_local_position_s		_local_pos {};		/**< vehicle local position */ //KITEX
	vehicle_rates_setpoint_s		_rates_sp {};		/* attitude rates setpoint */
	vehicle_status_s			_vehicle_status {};	/**< vehicle status */

	Subscription<airspeed_s>			_airspeed_sub;

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	float _flaps_applied{0.0f};
	float _flaperons_applied{0.0f};

	bool _landed{true};

	float _battery_scale{1.0f};

	bool _flag_control_attitude_enabled_last{false};

	struct {
		// path following 	// Kitex
		math::Vector<3> pos_c; 	// Kitex
		math::Vector<3> e_pi_x;	// Kitex
		math::Vector<3> e_pi_y;	// Kitex

		math::Vector<3> pos_b;	// Kitex

		float phiC;		// Kitex
		float thetaC;		// Kitex
		float turning_radius;	// Kitex

		float p_tc;
		float p_p;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float r_tc;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_ff;
		float y_integrator_max;
		float roll_to_yaw_ff;
		float y_rmax;

		bool w_en;
		float w_p;
		float w_i;
		float w_ff;
		float w_integrator_max;
		float w_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float dtrim_roll_vmin;
		float dtrim_pitch_vmin;
		float dtrim_yaw_vmin;
		float dtrim_roll_vmax;
		float dtrim_pitch_vmax;
		float dtrim_yaw_vmax;
		float dtrim_roll_flaps;
		float dtrim_pitch_flaps;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */
		float man_roll_scale;			/**< scale factor applied to roll actuator control in pure manual mode */
		float man_pitch_scale;			/**< scale factor applied to pitch actuator control in pure manual mode */
		float man_yaw_scale; 			/**< scale factor applied to yaw actuator control in pure manual mode */

		float acro_max_x_rate_rad;
		float acro_max_y_rate_rad;
		float acro_max_z_rate_rad;

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		float rattitude_thres;

		int32_t vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

		int32_t bat_scale_en;			/**< Battery scaling enabled */
		bool airspeed_disabled;

	} _parameters{};			/**< local copies of interesting parameters */

	struct {
		param_t phiC;		// Kitex
		param_t thetaC;		// Kitex
		param_t turning_radius;	// Kitex
	
		param_t x_pos_b;	// Kitex
		param_t y_pos_b;	// Kitex
		param_t z_pos_b;	// Kitex

		param_t p_tc;
		param_t p_p;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t r_tc;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_ff;
		param_t y_integrator_max;
		param_t roll_to_yaw_ff;
		param_t y_rmax;

		param_t w_en;
		param_t w_p;
		param_t w_i;
		param_t w_ff;
		param_t w_integrator_max;
		param_t w_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t dtrim_roll_vmin;
		param_t dtrim_pitch_vmin;
		param_t dtrim_yaw_vmin;
		param_t dtrim_roll_vmax;
		param_t dtrim_pitch_vmax;
		param_t dtrim_yaw_vmax;
		param_t dtrim_roll_flaps;
		param_t dtrim_pitch_flaps;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_roll_scale;
		param_t man_pitch_scale;
		param_t man_yaw_scale;

		param_t acro_max_x_rate;
		param_t acro_max_y_rate;
		param_t acro_max_z_rate;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t rattitude_thres;

		param_t vtol_type;

		param_t bat_scale_en;
		param_t airspeed_mode;

	} _parameter_handles{};		/**< handles for interesting parameters */

	math::Vector<3> _pos; // KiteX: local position
	math::Vector<3> _vel; // KiteX: local position

	float _pi_path_x[60]; // KiteX: x-coords of path points
	float _pi_path_y[60]; // KiteX: y-coords of path points
	int _pi_path_i = 0;   // KiteX: Path of kite in Pi plane

	math::Vector<2> _pos_pi; // KiteX: Projected position in Pi
	math::Vector<2> _vel_pi; // KiteX: Projected velocity vector in Pi
	math::Vector<2> _target_point_pi; // KiteX: Target point on path
	float _arc_radius = 100000000.0f; // KiteX: Radius of path
	float _arc_roll_rate = 0.0f; // KiteX: Yaw rate to reach path

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;

	void control_flaps(const float dt);

	void update_pi(float phi, float theta); 	// KiteX
	void update_pi_path(float radius);		// KiteX
	void update_pi_projection();			// KiteX
	void update_pi_target_point(float search_radius);// KiteX
	void update_pi_arc();				// KiteX
	void update_pi_roll_rate();			// KiteX
	// void publishTurning();			// KiteX

	void control_looping(float dt);			// KiteX

	// KiteX pure helpers
	float signed_angle(const math::Vector<2> &left, const math::Vector<2> &right);// KiteX
	float square_distance_to_path(const int path_i);// KiteX


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	void		vehicle_control_mode_poll();
	void		local_pos_poll();		// Kitex
	void		vehicle_manual_poll();
	void		vehicle_setpoint_poll();
	void		global_pos_poll();
	void		vehicle_status_poll();
	void		vehicle_land_detected_poll();

};
