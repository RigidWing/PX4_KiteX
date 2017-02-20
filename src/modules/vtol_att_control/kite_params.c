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
 * @file kite_params.c
 * Parameters for kite vtol attitude controller.
 *
 * @author Andreas Okholm   <bapstroman@gmail.com>
 */

#include <systemlib/param/param.h>

/**
 * Transition forward to FW mode roll target
 *
 * @unit rad
 * @min -1.6
 * @max 1.6
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
*/
PARAM_DEFINE_FLOAT(VT_T_F_ROLL, -1.4f);

/**
 * Transition forward to FW mode thrust target
 *
 * @unit normalized
 * @min 0.2
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
*/
PARAM_DEFINE_FLOAT(VT_T_F_THRUST, 0.8f);

/**
 * Transition backwards to MC mode pitch target
 *
 * @unit rad
 * @min -0.8
 * @max 0.8
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
*/
PARAM_DEFINE_FLOAT(VT_T_B_PITCH, -0.3f);

/**
 * Transition backwards to MC mode roll target
 *
 * @unit rad
 * @min -0.8
 * @max 0.8
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
*/
PARAM_DEFINE_FLOAT(VT_T_B_ROLL, 0.0f);

/**
 * Transition backwards to MC mode thrust target
 *
 * @unit normalized
 * @min 0.2
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
*/
PARAM_DEFINE_FLOAT(VT_T_B_THRUST, 0.4f);

/**
 * Windspeed
 *
 * @unit rad
 * @min 0
 * @max 20
 * @increment 0.1
 * @decimal 3
 * @group VTOL Attitude Control
*/
PARAM_DEFINE_FLOAT(VT_WIND_SPEED, 4f);
