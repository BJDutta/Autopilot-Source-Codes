/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file accelerometer_calibration.h
 *
 * Definition of accelerometer calibration.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef ACCELEROMETER_CALIBRATION_H_
#define ACCELEROMETER_CALIBRATION_H_

//#include <stdint.h>
#include "MATRIX.h"

#define CONSTANTS_ONE_G 9.81f
#define FLT_EPSILON 1.19209290e-07f

enum RESULT {
	OK = 1
};
enum detect_orientation_return {
	DETECT_ORIENTATION_TAIL_DOWN,
	DETECT_ORIENTATION_NOSE_DOWN,
	DETECT_ORIENTATION_LEFT,
	DETECT_ORIENTATION_RIGHT,
	DETECT_ORIENTATION_UPSIDE_DOWN,
	DETECT_ORIENTATION_RIGHTSIDE_UP,
	DETECT_ORIENTATION_ERROR
};
#define max_accel_sens 3
#define detect_orientation_side_count 6
#define CONSTANTS_ONE_G 9.81f

//void get_rot_matrix(MATRIX R);
int do_accel_calibration();
int calibrate_from_orientation(float accel_ref[detect_orientation_side_count][3]);
int read_accelerometer_avg(float accel_avg[detect_orientation_side_count][3], unsigned int orient, unsigned int samples_num);

int calculate_calibration_values(float accel_ref[detect_orientation_side_count][3], float accel_T[3][3], float accel_offs[3], float g);
int mat_invert3(float src[3][3], float dst[3][3]);

enum detect_orientation_return detect_orientation();
const char* detect_orientation_str(enum detect_orientation_return orientation);
#endif /* ACCELEROMETER_CALIBRATION_H_ */
