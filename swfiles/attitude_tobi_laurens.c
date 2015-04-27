/*
 * attitude_tobi_laurens.c
 *
 *  Created on: May 31, 2011
 *      Author: pixhawk
 */

/*
 * attitude_tobi_laurens.c
 *
 *  Created on: 21.12.2010
 *      Author: Laurens Mackay
 */

#include "imu.h"

#include "attitude_tobi_laurens.h"
#include "kalman.h"

#define SCA3100_COUNTS_PER_G 900


//#include "debug.h"
//#include "sensors.h"
#include "math.h"
//#include "conf.h"
//#include "altitude_speed.h"
#include "transformation.h"
//#include "gps_transformations.h"
//#include "pixhawk/mavlink.h"

//#define VELOCITY_HOLD 0.999f
//#define ACCELERATION_HOLD 0.99f
//#define VELOCITY_HOLD 1.0f
//#define ACCELERATION_HOLD 1.0f
#define TIME_STEP (1.0f / 200.0f)		// change for higher calculation speeds

kalman_t attitude_tobi_laurens_kal;

void vect_norm(float_vect3 *vect)
{
	float length = sqrtf(
			vect->x * vect->x + vect->y * vect->y + vect->z * vect->z);
	if (length != 0)
	{
		vect->x /= length;
		vect->y /= length;
		vect->z /= length;
	}
}



void vect_cross_product(const float_vect3 *a, const float_vect3 *b,
		float_vect3 *c)
{
	c->x = a->y * b->z - a->z * b->y;
	c->y = a->z * b->x - a->x * b->z;
	c->z = a->x * b->y - a->y * b->x;
}

void attitude_tobi_laurens_update_a(void)
{
	// for acc
	// Idendity matrix already in A.
	M(attitude_tobi_laurens_kal.a, 0, 1) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 11);
	M(attitude_tobi_laurens_kal.a, 0, 2) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 10);

	M(attitude_tobi_laurens_kal.a, 1, 0) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 11);
	M(attitude_tobi_laurens_kal.a, 1, 2) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 9);

	M(attitude_tobi_laurens_kal.a, 2, 0) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 10);
	M(attitude_tobi_laurens_kal.a, 2, 1) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 9);

	// for mag
	// Idendity matrix already in A.
	M(attitude_tobi_laurens_kal.a, 3, 4) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 11);
	M(attitude_tobi_laurens_kal.a, 3, 5) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 10);

	M(attitude_tobi_laurens_kal.a, 4, 3) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 11);
	M(attitude_tobi_laurens_kal.a, 4, 5) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 9);

	M(attitude_tobi_laurens_kal.a, 5, 3) = TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 10);
	M(attitude_tobi_laurens_kal.a, 5, 4) = -TIME_STEP * kalman_get_state(
			&attitude_tobi_laurens_kal, 9);

}

void attitude_tobi_laurens_init(void)
{
	//X Kalmanfilter
	//initalize matrices

	static m_elem kal_a[12 * 12] =
	{ 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0f, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0f };

	static m_elem kal_c[9 * 12] =
	{ 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 1.0f, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 1.0f, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 1.0f };



#define FACTOR 0.5f
#define FACTORstart 1.0f


	static m_elem kal_gain[12 * 9] =
	{ 		0.004f , 0    ,   0    ,   0    ,   0    ,   0    ,   0   ,    0    ,   0,
			0   ,    0.004f , 0   ,    0   ,    0   ,    0   ,    0   ,    0   ,    0,
			0   ,    0    ,   0.004f , 0   ,    0   ,    0   ,    0   ,    0   ,    0,
			0   ,    0    ,   0   ,    0.06f, 	0   ,    0   ,    0   ,    0   ,    0,
			0   ,    0   ,    0   ,    0    ,   0.06f, 	 0   ,    0   ,    0   ,    0,
			0   ,    0    ,   0   ,    0    ,   0   ,    0.06f, 	  0   ,    0   ,    0,
			0.0000f , +0.0000002f,0   ,    0 , 		0, 		 0,  	  0,  	   0    ,   0,
			-0.0000002f,0    ,   0   ,    0 , 		0, 		 0,  	  0,  	   0, 	    0,
			0,    	 0 ,	  0   ,    0,  	    0,		 0,  	  0,  	   0, 	    0,
			0  ,     0    ,   0   ,    0   ,    0    ,   0   ,    0.9f ,   0   ,    0,
			0   ,    0   ,    0   ,    0   ,    0    ,   0   ,    0    ,   0.9f ,   0,
			0   ,    0   ,    0   ,    0   ,    0   ,    0   ,    0    ,   0    ,   0.9f
	};


#define K 10.0f*TIME_STEP

	static m_elem kal_gain_start[12 * 9] =
	{ K, 0, 0, 0, 0, 0, 0, 0, 0,

			0, K, 0, 0, 0, 0, 0, 0, 0,

			0, 0, K, 0, 0, 0, 0, 0, 0,

			0, 0, 0, K, 0, 0, 0, 0, 0,

			0, 0, 0, 0, K, 0, 0, 0, 0,

			0, 0, 0, 0, 0, K, 0, 0, 0,

			0,0.00008f, 0, 0, 0, 0, 0, 0, 0,

			-0.00008f ,0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, K,

			0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0,

			0, 0, 0, 0, 0, 0, 0, 0, 0 };



	static m_elem kal_x_apriori[12 * 1] =
	{  };


	//---> initial states sind aposteriori!? ---> fehler
	static m_elem kal_x_aposteriori[12 * 1] =
	{ 0, 0, -9.81f, 0.f, -0.2f, -0.9f, 0, 0, 0, 0, 0, 0 };

	kalman_init(&attitude_tobi_laurens_kal, 12, 9, kal_a, kal_c,
			kal_gain_start, kal_gain, kal_x_apriori, kal_x_aposteriori, 1000);

}

void attitude_tobi_laurens(void)
{
	//Transform accelerometer used in all directions
	//	float_vect3 acc_nav;
	//body2navi(&global_data.accel_si, &global_data.attitude, &acc_nav);

	// Kalman Filter

	//Calculate new linearized A matrix
	attitude_tobi_laurens_update_a();

	kalman_predict(&attitude_tobi_laurens_kal);

	//correction update

	m_elem measurement[9] =
	{ };
	m_elem mask[9] =
	{ 1.0f, 1.0f, 1.0f, 0, 0, 0, 1.0f, 1.0f, 1.0f };

	float_vect3 acc_v;
	float_vect3 mag_v;
	float_vect3 gyro_v;

	//	acc.x = global_data.accel_raw.x * 9.81f /690;
	//	acc.y = global_data.accel_raw.y * 9.81f / 690;
	//	acc.z = global_data.accel_raw.z * 9.81f / 690;



	acc_v.x = acc[0];
	acc_v.y = acc[1];
	acc_v.z = acc[2];



	float acc_norm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])/9.81*SCA3100_COUNTS_PER_G;
	static float acc_norm_filt = SCA3100_COUNTS_PER_G;
	float acc_norm_lp = 0.05f;
	acc_norm_filt = (1.0f - acc_norm_lp) * acc_norm_filt + acc_norm_lp
			* acc_norm;

	//	static float acc_norm_filtz = SCA3100_COUNTS_PER_G;
	//	float acc_norm_lpz = 0.05;
	//	acc_norm_filtz = (1.0f - acc_norm_lpz) * acc_norm_filtz + acc_norm_lpz * -acc.z;

	float acc_diff = fabs(acc_norm_filt - SCA3100_COUNTS_PER_G);
	if (acc_diff > 200.0f)
	{
		//Don't correct when acc high
		mask[0] = 0;
		mask[1] = 0;
		mask[2] = 0;

	}
	else if (acc_diff > 100.0f)
	{
		//fade linearely out between 100 and 200
		float mask_lin = (200.0f - acc_diff) / 100.0f;
		mask[0] = mask_lin;
		mask[1] = mask_lin;
		mask[2] = mask_lin;
	}
	//else mask stays 1

	//	static uint8_t i = 0;
	//	if (i++ > 10)
	//	{
	//		i = 0;
	//		float_vect3 debug;
	//		debug.x = mask[0];
	//		debug.y = acc_norm;
	//		debug.z = acc_norm_filt;
	//		debug_vect("acc_norm", debug);
	//	}

	//	mag.x = (global_data.magnet_corrected.x ) * 1.f / 510.f;
	//	mag.y = (global_data.magnet_corrected.y) * 1.f / 510.f;
	//	mag.z = (global_data.magnet_corrected.z) * 1.f / 510.f;
	mag_v.x = mag[0] ;
	mag_v.y = mag[1];
	mag_v.z = mag[2] ;


	//	gyro.x = -(global_data.gyros_raw.x-global_data.param[PARAM_GYRO_OFFSET_X]) * 0.000955;
	//	gyro.y = (global_data.gyros_raw.y-global_data.param[PARAM_GYRO_OFFSET_Y]) * 0.000955;
	//	gyro.z = -(global_data.gyros_raw.z-global_data.param[PARAM_GYRO_OFFSET_Z]) * 0.001010;

	gyro_v.x = gyro[0];
	gyro_v.y = gyro[1];
	gyro_v.z = gyro[2];



	measurement[0] = acc_v.x;
	measurement[1] = acc_v.y;
	measurement[2] = acc_v.z;


	measurement[3] = mag_v.x;
	measurement[4] = mag_v.y;
	measurement[5] = mag_v.z;

	measurement[6] = gyro_v.x;
	measurement[7] = gyro_v.y;
	measurement[8] = gyro_v.z;

	//Put measurements into filter


	static int j = 0;

	// MASK

	if (j >= 3)
	{
		j = 0;

		mask[3]=1;
		mask[4]=1;
		mask[5]=1;
	}
	else
	{
		j++;
	}


	kalman_correct(&attitude_tobi_laurens_kal, measurement, mask);


	//debug

	// save outputs
	float_vect3 kal_acc, kal_mag, kal_w0, kal_w;

	kal_acc.x = kalman_get_state(&attitude_tobi_laurens_kal, 0);
	kal_acc.y = kalman_get_state(&attitude_tobi_laurens_kal, 1);
	kal_acc.z = kalman_get_state(&attitude_tobi_laurens_kal, 2);

	kal_mag.x = kalman_get_state(&attitude_tobi_laurens_kal, 3);
	kal_mag.y = kalman_get_state(&attitude_tobi_laurens_kal, 4);
	kal_mag.z = kalman_get_state(&attitude_tobi_laurens_kal, 5);

	kal_w0.x = kalman_get_state(&attitude_tobi_laurens_kal, 6);
	kal_w0.y = kalman_get_state(&attitude_tobi_laurens_kal, 7);
	kal_w0.z = kalman_get_state(&attitude_tobi_laurens_kal, 8);

	kal_w.x = kalman_get_state(&attitude_tobi_laurens_kal, 9);
	kal_w.y = kalman_get_state(&attitude_tobi_laurens_kal, 10);
	kal_w.z = kalman_get_state(&attitude_tobi_laurens_kal, 11);

	float_vect3 x_n_b, y_n_b, z_n_b;
	z_n_b.x = -kal_acc.x;
	z_n_b.y = -kal_acc.y;
	z_n_b.z = -kal_acc.z;
	vect_norm(&z_n_b);
	vect_cross_product(&z_n_b, &kal_mag, &y_n_b);
	vect_norm(&y_n_b);

	vect_cross_product(&y_n_b, &z_n_b, &x_n_b);

	//save euler angles
	imu.roll = R2D*atan2f(z_n_b.y, z_n_b.z);
	imu.pitch = R2D*-asinf(z_n_b.x);

	//		static hackMagLowpass = 0.0f;
	imu.yaw = imu.yaw*0.9f+0.1f*R2D*atan2f(y_n_b.x, x_n_b.x);


	//save rates
	imu.rollrate = R2D*kal_w.x;
	imu.pitchrate = R2D*kal_w.y;
	imu.yawrate = R2D*kal_w.z;

//	global_data.yaw_lowpass = 0.99f * global_data.yaw_lowpass + 0.01f * global_data.attitude_rate.z;
}
