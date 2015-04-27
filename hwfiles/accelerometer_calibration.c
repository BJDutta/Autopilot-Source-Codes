#include "accelerometer_calibration.h"
#include "imu.h"
#include "config.h"
#include "lpc17xx_wdt.h"
#include "param.h"
#include "i2ceeprom.h"
#include "mlink.h"
#include "lpc17xx_timer.h"
#include "quat.h"
#include "MPU6050.h"
#include "string.h"
#include "lpc_types.h"
#include "math.h"
#include "timing.h"
#include "MATRIX.h"
#include "mlink.h"
#include "comm.h"
#include "LED_Blink.h"


//void get_rot_matrix(MATRIX R){
//
//
//
//
//
//
//	R[0][0] = 	1 - 2*qy2 - 2*qz2; 	R[0][1] = 2*qx*qy - 2*qz*qw;	R[0][2] = 2*qx*qz + 2*qy*qw;
//	R[1][0] = 	2*qx*qy + 2*qz*qw;	R[1][1] = 1 - 2*qx2 - 2*qz2;	R[1][2] = 2*qy*qz - 2*qx*qw;
//	R[2][0] = 	2*qx*qz - 2*qy*qw;	R[2][1] = 2*qy*qz + 2*qx*qw;	R[2][2] = 1 - 2*qx2 - 2*qy2;
//
//
//}g;

int do_accel_calibration()
{
	float accel_scale_final[3];
	int i;
	int res = OK;
	char str[50];
	float accel_offs[3];
	float accel_T[3][3];
	float accel_ref[detect_orientation_side_count][3];

	for(i=0; i<50; i++)str[i]= 0;
	strcpy(str, "Accelerometer Calibration started");
	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str);tel_transmit();



	if (res == OK) {
		/* measure and calculate offsets & scales */
		//		res = do_accel_calibration_measurements(accel_offs, accel_T);
		res = calibrate_from_orientation(accel_ref);
		res = calculate_calibration_values(accel_ref, accel_T, accel_offs, CONSTANTS_ONE_G);

	}

	if (res != OK ) {

		return ERROR;  // send a mavlink message to user also
	}


	//	//	/* measurements completed successfully, rotate calibration values */
	//
	//
	//	MATRIX board_rotation = mat_creat(3,3,UNIT_MATRIX);		// free the memory
	//	get_rot_matrix(&board_rotation);					// make this function to find rotation matrix
	//	MATRIX board_rotation_t = mat_creat(3,3,UNIT_MATRIX);	// free the memory
	//	mat_tran1(board_rotation_t,board_rotation);
	//
	//	MATRIX temp33 = mat_creat(3,3, UNIT_MATRIX);		// free the memory
	//
	//
	//	//	for (unsigned i = 0; i < active_sensors; i++) {
	//
	//	//	/* handle individual sensors, one by one */
	//	//	math::Vector<3> accel_offs_vec(accel_offs);
	//	//	math::Vector<3> accel_offs_rotated = board_rotation_t * accel_offs_vec;
	//
	//	MATRIX accel_offs_rotated = mat_creat(3,1,ZERO_MATRIX);	// free the memory
	//
	//	mat_mul1(accel_offs_rotated, board_rotation_t, accel_offs);
	//
	//
	//	//	math::Matrix<3, 3> accel_T_mat(accel_T);
	//	//	math::Matrix<3, 3> accel_T_rotated = board_rotation_t * accel_T_mat * board_rotation;
	//
	//
	//	MATRIX accel_T_rotated = mat_creat(3,3,UNIT_MATRIX);   // free the memory
	//	mat_mul1(temp33, board_rotation_t,accel_T);			// check with debugger for valid matrix type of accel_T as float array
	//	mat_mul1(accel_T_rotated, temp33, board_rotation);


	//	accel_offs[0] = accel_offs_rotated[0];
	//	accel_scale_final[0] = accel_T_rotated[0][0];
	//	accel_offs[1] = accel_offs_rotated[1];
	//	accel_scale_final[1] = accel_T_rotated[1][1];
	//	accel_offs[2] = accel_offs_rotated[2];
	//	accel_scale_final[2] = accel_T_rotated[2][2];
	//	}

	accel_offs[0] = accel_offs[0];
	accel_scale_final[0] = accel_T[0][0];
	accel_offs[1] = accel_offs[1];
	accel_scale_final[1] = accel_T[1][1];
	accel_offs[2] = accel_offs[2];
	accel_scale_final[2] = accel_T[2][2];

	*AccoffX = (float)accel_offs[0]; *AccoffY = (float)accel_offs[1]; *AccoffZ = (float)accel_offs[2];
	//	*GyrooffX = (float)gyro_off[0]; *GyrooffY = (float)gyro_off[1]; *GyrooffZ = (float)gyro_off[2];
	*MagoffX = (float)accel_scale_final[0]; *MagoffY = (float)accel_scale_final[1]; *MagoffZ = (float)accel_scale_final[2];


	//	mat_free(board_rotation);
	//	mat_free(board_rotation_t);
	//	mat_free(temp33);
	//	mat_free(accel_offs_rotated);
	//	mat_free(accel_T_rotated);


	eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));

	for(i=0; i<50; i++)str[i]= 0;
	strcpy(str,"Calibration Successful- Rebooting!!" );
	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str);tel_transmit();

	delay(2000);
	NVIC_SystemReset();

	return res;
}


int calibrate_from_orientation(float accel_ref[detect_orientation_side_count][3])
{
	int result = OK;
	unsigned int i=0;
	//	char stra[50], strb[50], strc[50];
	//	char strd[50], stre[50];
	char str[50];


	Bool side_data_collected[detect_orientation_side_count] = { FALSE,FALSE,FALSE,FALSE,FALSE,FALSE, };
	unsigned orientation_failures = 0;

	// Rotate through all three main positions
	while (TRUE) {
		if (orientation_failures > 10) {
			result = ERROR;

			for(i=0; i<50; i++)str[i]= 0;
			strcpy(str,"Calibration Failed - Timeout" );


			mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str);tel_transmit();
			// mavlink_and_console_log_info(mavlink_fd, CAL_FAILED_ORIENTATION_TIMEOUT);
			break;
		}

		unsigned int side_complete_count = 0;
		unsigned int cur_orientation = 0;

		// Update the number of completed sides
		for (i = 0; i < detect_orientation_side_count; i++) {
			if (side_data_collected[i]) {
				side_complete_count++;
			}
		}

		if (side_complete_count == detect_orientation_side_count) {
			// We have completed all sides, move on
			break;
		}

		/* inform user which orientations are still needed */

		if(side_complete_count!=0){
			for(i=0; i<50; i++)str[i] = 0;
			for (cur_orientation=0; cur_orientation<detect_orientation_side_count; cur_orientation++) {
				if (!side_data_collected[cur_orientation]) {
					strcat(str, detect_orientation_str((enum detect_orientation_return)cur_orientation));	//detect_orientation_return(orient) - this could eliminate the printing of all the possible orientations
					strcat(str, ", ");
				}
			}

			strcat(str, " - Pending");
			mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str); tel_transmit();
		}


		//		for(i=0; i<50; i++)strb[i]= 0;
		//		strcpy(strb, pendingStr);

		//		mavlink_and_console_log_info(mavlink_fd, "pending:%s", pendingStr);

		//		mavlink_and_console_log_info(mavlink_fd, "hold the vehicle still on one of the pending sides");
		enum detect_orientation_return orient = detect_orientation();

		if (orient == DETECT_ORIENTATION_ERROR) {
			orientation_failures++;
			for(i=0; i<50; i++)str[i]= 0;
			strcpy(str, "Cannot detect Orientation");
			mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str);tel_transmit();
			//			mavlink_and_console_log_info(mavlink_fd, "detected motion, hold still...");
			continue;
		}

		/* inform user about already handled side */
		if (side_data_collected[orient]) {
			orientation_failures++;
			//			mavlink_and_console_log_info(mavlink_fd, "%s side already completed or not needed", detect_orientation_str(orient));
			//			mavlink_and_console_log_info(mavlink_fd, "rotate to a pending side");

			for(i=0; i<50; i++)str[i]= 0;
			strcat(str, detect_orientation_str(orient));
			strcat(str, " side already done, rotate to another side");

			mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str);tel_transmit();
			//			mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, ",\n rotate to pending side");
			continue;
		}

		//		mavlink_and_console_log_info(mavlink_fd, "%s orientation detected", detect_orientation_str(orient));


		for(i=0; i<50; i++)str[i]= 0;
		strcat(str, detect_orientation_str(orient));
		strcat(str, " side detected, hold still. Taking readings!!");
		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str);tel_transmit();

		orientation_failures = 0;

		read_accelerometer_avg(accel_ref, orient, 50);

		//		mavlink_and_console_log_info(mavlink_fd, "%s side done, rotate to a different side", detect_orientation_str(orient));

		for(i=0; i<50; i++)str[i]= 0;
		strcat(str, detect_orientation_str(orient));
		strcat(str, " side done, proceed to next side");

		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str);tel_transmit();  // commented since it looks more like a repetition


		// Note that this side is complete
		side_data_collected[orient] = TRUE;
		delay(4000);
	}


	return result;
}

int read_accelerometer_avg(float accel_avg[detect_orientation_side_count][3], unsigned int orient, unsigned int samples_num)
{

	float accel_sum[3];
	int16_t accel_imu_temp[3];
	int i,j;

	//	memset(accel_sum, 0, sizeof(accel_sum));

	for(i=0; i<samples_num; i++){
		delay(40);
		WDT_Feed();
		blink();
		MPU6050_GetRawAccel(accel_imu_temp);
		for(j=0;j<3;j++)
		{
			accel_sum[j]+=accel_imu_temp[j];
		}
	}
	for (i = 0; i < 3; i++) {
		accel_avg[orient][i] = accel_sum[i] / (float)samples_num;
	}


	return OK;

}

int calculate_calibration_values(float accel_ref[detect_orientation_side_count][3], float accel_T[3][3], float accel_offs[3], float g)
{
	unsigned int i=0, j=0;
	/* calculate offsets */
	for (i = 0; i < 3; i++) {
		accel_offs[i] = (accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i]) / 2;
	}

	/* fill matrix A for linear equations system*/
	float mat_A[3][3];
	//	memset(mat_A, 0, sizeof(mat_A));

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			float a = accel_ref[i * 2][j] - accel_offs[j];
			mat_A[i][j] = a;
		}
	}

	/* calculate inverse matrix for A */
	float mat_A_inv[3][3];

	if (mat_invert3(mat_A, mat_A_inv) != OK) {
		return ERROR;
	}

	/* copy results to accel_T */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			/* simplify matrices mult because b has only one non-zero element == g at index i */
			accel_T[j][i] = mat_A_inv[j][i] * g;
		}
	}

	return OK;
}

int mat_invert3(float src[3][3], float dst[3][3])
{
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
			src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
			src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	if (fabsf(det) < FLT_EPSILON) {
		return ERROR;        // Singular matrix
	}

	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

	return OK;
}

enum detect_orientation_return detect_orientation()
{
#define ndim 3


	/* exponential moving average of accel */
	float accel_ema[ndim] = { 0.0f };
	/* max-hold dispersion of accel */
	float accel_disp[3] = { 0.0f, 0.0f, 0.0f };
	/* EMA time constant in seconds*/
	float ema_len = 0.5f;
	/* set "still" threshold to 0.25 m/s^2 */
	float still_thr2 = powf(0.25f, 2);
	/* set accel error threshold to 5m/s^2 */
	float accel_err_thr = 5.0f;
	/* still time required in us */
	uint64_t still_time = 2000000;


	uint64_t t_start = micros();
	/* set timeout to 19s */
	uint64_t timeout = 19000000;
	uint64_t t_timeout = t_start + timeout;
	uint64_t t = t_start;
	uint64_t t_prev = t_start;
	uint64_t t_still = 0;

	unsigned int poll_errcount = 0;
	unsigned int i=0;
	int16_t acc_local[3];

	char str[50];

	while (TRUE) {
		/* wait blocking for new data */
		int poll_ret = 1;
		poll_ret = MPU6050_GetRawAccel(acc_local);
		if (poll_ret) {

			t = micros();
			float dt = (t - t_prev) / 1000000.0f;
			t_prev = t;
			float w = dt / ema_len;

			for (i = 0; i < ndim; i++) {

				float di = 0.0f;
				switch (i) {
				case 0:
					di = (9.81f*(float)acc_local[0])/4096;
					break;
				case 1:
					di = (9.81f*(float)acc_local[1])/4096;
					break;
				case 2:
					di = (9.81f*(float)acc_local[2])/4096;
					break;
				}

				float d = di - accel_ema[i];
				accel_ema[i] += d * w;
				d = d * d;
				accel_disp[i] = accel_disp[i] * (1.0f - w);

				if (d > still_thr2 * 8.0f) {
					d = still_thr2 * 8.0f;
				}

				if (d > accel_disp[i]) {
					accel_disp[i] = d;
				}
			}

			/* still detector with hysteresis */
			if (accel_disp[0] < still_thr2 &&
					accel_disp[1] < still_thr2 &&
					accel_disp[2] < still_thr2) {
				/* is still now */
				if (t_still == 0) {
					/* first time */
					//					for(i=0; i<50; i++)str[i]= 0;
					//					strcat(str, "Side, keep stable");
					//					mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str); tel_transmit();
					//					mavlink_and_console_log_info(mavlink_fd, "detected rest position, hold still...");
					t_still = t;
					t_timeout = t + timeout;

				} else {
					/* still since t_still */
					if (t > t_still + still_time) {
						/* vehicle is still, exit from the loop to detection of its orientation */
						break;
					}
				}

			} else if (accel_disp[0] > still_thr2 * 4.0f ||
					accel_disp[1] > still_thr2 * 4.0f ||
					accel_disp[2] > still_thr2 * 4.0f) {
				/* not still, reset still start time */
				if (t_still != 0) {

					for(i=0; i<50; i++)str[i]= 0;
					strcat(str, "Shakes detected - hold still, retrying");
					mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, &str); tel_transmit();
					//					mavlink_and_console_log_info(mavlink_fd, "detected motion, hold still...");
					delay(3000);
					t_still = 0;
				}
			}

		} else if (poll_ret == 0) {
			poll_errcount++;
		}

		if (t > t_timeout) {
			poll_errcount++;
		}

		if (poll_errcount > 1000) {

			//			mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, "Calibration Failed"); tel_transmit();
			//			mavlink_and_console_log_critical(mavlink_fd, CAL_FAILED_SENSOR_MSG);
			return DETECT_ORIENTATION_ERROR;
		}
	}

	if (fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr &&
			fabsf(accel_ema[1]) < accel_err_thr &&
			fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_TAIL_DOWN;
		//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_NOTICE, "Tail Down");	tel_transmit();// [ g, 0, 0 ]
	}

	if (fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr &&
			fabsf(accel_ema[1]) < accel_err_thr &&
			fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_NOSE_DOWN;
		//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_NOTICE, "Nose Down");	tel_transmit();// [ -g, 0, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
			fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr &&
			fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_LEFT;
		//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_NOTICE, "Left");tel_transmit();	// [ 0, g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
			fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr &&
			fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_RIGHT;
		//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_NOTICE, "Right");tel_transmit();	// [ 0, -g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
			fabsf(accel_ema[1]) < accel_err_thr &&
			fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr) {
		return DETECT_ORIENTATION_UPSIDE_DOWN;
		//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_NOTICE, "Upside Down");tel_transmit();	// [ 0, 0, g ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
			fabsf(accel_ema[1]) < accel_err_thr &&
			fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr) {
		return DETECT_ORIENTATION_RIGHTSIDE_UP;
		//		mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_NOTICE, "Rightside Up");tel_transmit();// [ 0, 0, -g ]
	}

	//	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, "Invalid Orientation");tel_transmit();
	//	mavlink_and_console_log_critical(mavlink_fd, "ERROR: invalid orientation");

	return DETECT_ORIENTATION_ERROR;
	//	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_ALERT, "Cannot detect orientation");tel_transmit();// Can't detect orientation
}

const char* detect_orientation_str(enum detect_orientation_return orientation)
{
	static const char* rgOrientationStrs[] = {
			"BACK",		// tail down
			"FRONT",	// nose down
			"LEFT",
			"RIGHT",
			"UP",		// upside-down
			"DOWN",		// right-side up
			"ERROR"
	};

	return rgOrientationStrs[orientation];
}
