#include "quat.h"
#include "mlink.h"
#include "gps.h"
#include "control.h"

//float *IMU_GAIN = (float *) &param_array[IMU_GAIN_IDX];

float gyro[3];
float acc[3];
float mag[3];

float roll,pitch,yaw;
float a_roll,a_pitch,a_yaw;

float attitude[3];
float acc_attitude[3];

float accvel=0;
float accvel1=0;
float accvel2=0;

float q[4];
float qj[4];
float qa[4];

ins_t INS;

float AX_E, AY_E, AZ_E;


//! Auxiliary variables to reduce number of repeated operations
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f, qw = 0.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */

static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;


//#define gx gyro[0]
//#define gy gyro[1]
//#define gz gyro[2]

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float number)
{
	volatile long i;
	volatile float x, y;
	volatile const float f = 1.5F;

	x = number * 0.5F;
	y = number;
	i = * (( long * ) &y);
	i = 0x5f375a86 - ( i >> 1 );
	y = * (( float * ) &i);
	y = y * ( f - ( x * y * y ) );
	return y;
}

void init_quat()
{

	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;
	float initialHdg, cosHeading, sinHeading;

	initialRoll = atan2(acc[1], acc[2]);  //roll along x
	initialPitch = atan2(acc[0], acc[2]);  // pitch along y

	//	acc_attitude[1] = R2D*atan(acc[0]/acc[2]);	// apitch
	//	acc_attitude[0] = R2D*atan2(acc[1], acc[2]);	// aroll

	cosRoll = cosf(initialRoll);  // giving component form
	sinRoll = sinf(initialRoll);  // giving component form
	cosPitch = cosf(initialPitch);  // giving component form
	sinPitch = sinf(initialPitch);  // giving component form

	magX = mag[0] * cosPitch + mag[1] * sinRoll * sinPitch + mag[2] * cosRoll * sinPitch;

	magY = mag[1] * cosRoll - mag[2] * sinRoll;

	//	initialHdg = atan2f(-magY, magX);	// disabling the magnetometer
	initialHdg = atan2f(-magY, magX);	// disabling the magnetometer

	initialHdg = 0;

	cosRoll = cosf(initialRoll * 0.5f);
	sinRoll = sinf(initialRoll * 0.5f);

	cosPitch = cosf(initialPitch * 0.5f);
	sinPitch = sinf(initialPitch * 0.5f);

	cosHeading = cosf(initialHdg * 0.5f);
	sinHeading = sinf(initialHdg * 0.5f);

	//to help conversion from body to earth frame --- http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	q[0] = q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	q[1] = q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	q[2] = q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	q[3] = q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	// auxillary variables to reduce number of repeated operations, for 1st pass
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	INS.X = 0;
	INS.Y = 0;
	INS.Z = 0;
	INS.Vx = 0;
	INS.Vy = 0;
	INS.Vz = 0;

}

float constrain(float var, float llimit, float ulimit)
{
	if(var>ulimit)
		var=ulimit;
	else if(var<llimit)
		var = llimit;
	return var;
}

void mult_quat(float *ab, float *a, float *b)
{
	ab[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
	ab[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
	ab[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
	ab[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void conj_quat(float *qj, float *q)
{
	qj[0] = q[0];
	qj[1] = -q[1];
	qj[2] = -q[2];
	qj[3] = -q[3];
}

void unit_quat(float *q)
{
	float norm =  sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0]/norm;
	q[1] = q[1]/norm;
	q[2] = q[2]/norm;
	q[3] = q[3]/norm;
}

float norm_quat(float *q)
{
	float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	return norm;
}

void quat2angle(float *r, float *q)
{
	float rho, sinrho;
	if (fabs(q[0]) > 1)
		q[0] /= fabs(q[0]);
	rho = acos(q[0]);
	sinrho = sin(rho);
	r[0] = rho*2;		// theta, angle of rotation
	if(sinrho==0)
	{
		r[1] = 0;
		r[2] = 0;
		r[3] = 1;
	}
	else
	{
		r[1] = q[1] / sinrho;	// unit vector
		r[2] = q[2] / sinrho;
		r[3] = q[3] / sinrho;
	}
}

void angle2quat(float *q, float *r)
{
	float rho, sinrho;
	rho = r[0]/2;
	sinrho = sin(rho);

	q[0] = cos(rho);
	q[1] = r[1]*sinrho;
	q[2] = r[2]*sinrho;
	q[3] = r[3]*sinrho;
}

void quat2euler(float *angles, float *q)
{
	float xi, theta, rho;
	float q1,q2,q3,q4;
	q1 = q[0]; q2 = -q[1]; q3 = -q[2]; q4 = -q[3];

	xi = atan2(2*q2*q3 - 2*q1*q4, 2*(q1*q1) + 2*(q2*q2) - 1);
	theta = -asin(2*q2*q4 + 2*q1*q3);
	rho = atan2(2*q3*q4 - 2*q1*q2, 2*(q1*q1) + 2*(q4*q4) - 1);

	angles[0] = R2D*rho;
	angles[1] = R2D*theta;
	angles[2] = R2D*xi;
}

void euler2quat(float *q, float roll, float pitch, float yaw)
{
	float cosrho,costheta,cosxi;
	float sinrho,sintheta,sinxi;
	yaw= OptitrackHeading;

	float Qx[4], Qy[4], Qz[4], Qtemp[4];

	cosrho = cos(D2R*roll/2); costheta = cos(D2R*pitch/2); cosxi = cos(D2R*yaw/2);
	sinrho = sin(D2R*roll/2); sintheta = sin(D2R*pitch/2); sinxi = sin(D2R*yaw/2);

	Qx[0] = cosrho; Qx[1] = sinrho; Qx[2] = 0;  Qx[3] = 0;
	Qy[0] = costheta; Qy[1] = 0; Qy[2] = sintheta;  Qy[3] = 0;
	Qz[0] = cosxi; Qz[1] = 0; Qz[2] = 0;  Qz[3] = sinxi;

	mult_quat(Qtemp, Qz, Qy);
	mult_quat(q, Qtemp, Qx);
	unit_quat(q);
}

void integrate_gyro(float dt, char mergequat)
{
	float recipNorm, norm;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	float mx,my,mz,ax,ay,az,gx,gy,gz;
	float twoKp = 1, twoKi = 0.05;

	mx = mag[0]; my = mag[1]; mz = mag[2];
	ax = -acc[0]; ay = acc[1]; az = acc[2];
	gx = gyro[0]; gy = gyro[1]; gz = gyro[2];



	//	//! If magnetometer measurement is available, use it.
	//	if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
	//		float hx, hy, hz, bx, bz;
	//		float halfwx, halfwy, halfwz;
	//
	//		// Normalise magnetometer measurement
	//		// Will sqrt work better? PX4 system is powerful enough?
	//		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	//		mx *= recipNorm;
	//		my *= recipNorm;
	//		mz *= recipNorm;
	//
	//		// Reference direction of Earth's magnetic field
	//		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
	//		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
	//		hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
	//		bx = sqrt(hx * hx + hy * hy);
	//		bz = hz;
	//
	//		// Estimated direction of magnetic field
	//		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
	//		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
	//		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
	//
	//		// Error is sum of cross product between estimated direction and measured direction of field vectors
	//		halfex += (my * halfwz - mz * halfwy);
	//		halfey += (mz * halfwx - mx * halfwz);
	//		halfez += (mx * halfwy - my * halfwx);
	//	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float halfvx, halfvy, halfvz;

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += ay * halfvz - az * halfvy;
		halfey += az * halfvx - ax * halfvz;
		halfez += ax * halfvy - ay * halfvx;
	}

	norm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])/9.81;  // acceleration cut off at 0.75G < A < 1.25G
	if(norm < .75 || norm > 1.25){
		twoKp = 0;
		twoKi = 0;
	}
	else {
		twoKp = twoKp*constrain((-4*fabs(norm -1) + 1), 0, 1);
		twoKi = twoKi*constrain((-4*fabs(norm -1) + 1), 0, 1);
	}


	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if(twoKi >= 0.0f) {
			gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
			gyro_bias[1] += twoKi * halfey * dt;
			gyro_bias[2] += twoKi * halfez * dt;

			// apply integral feedback
			gx += gyro_bias[0];
			gy += gyro_bias[1];
			gz += gyro_bias[2];
		}
		else {
			gyro_bias[0] = 0.0f;	// prevent integral windup
			gyro_bias[1] = 0.0f;
			gyro_bias[2] = 0.0f;
		}


		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	//! Integrate rate of change of quaternion
#if 0
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
#endif

	// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
	//! q_k = q_{k-1} + dt*\dot{q}
	//! \dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx);

	q0 += dt*dq0;
	q1 += dt*dq1;
	q2 += dt*dq2;
	q3 += dt*dq3;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;

}

void merge_quats(float *q, float *qe, float *qa, float gain)
{
	float re[4],ra[4],r[4], qdir[4], norm;
	uint8_t i;

	for(i=0; i<4; i++)
		qdir[i] = qa[i] - qe[i];

	if(norm_quat(qdir) >= 1)
	{
		for(i=0; i<4; i++)
			qa[i] = -qa[i];
	}

	quat2angle(re,qe);
	quat2angle(ra,qa);

	norm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])/9.81;  // acceleration cut off at 0.6G < A < 1.4G
	if(norm < .6 || norm > 1.4) gain = 0;
	else {
		gain = gain*constrain((-2.5*fabs(norm -1) + 1), 0, 1);
	}


	for(i=0; i<4; i++)
		r[i] = re[i] + (ra[i]-re[i])*gain;
	norm = sqrt(r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);
	for(i=1; i<4; i++)
		r[i] = r[i]/norm;
	angle2quat(q, r);
}

void acc_angles()
{
	float a = acc[0]/9.81;
	float headY, headX, cos_pitch, sin_pitch, cos_roll, sin_roll;

	a = constrain(a, -1,1);
	//	acc_attitude[1] = R2D*asin(a);		// apitch
	if(acc[2]==0)acc[2]=0.01;

	acc_attitude[1] = R2D*atan(acc[0]/acc[2]);	// apitch
	acc_attitude[0] = R2D*atan2(acc[1], acc[2]);	// aroll

	cos_roll = cos(D2R*attitude[0]);
	sin_roll = sin(D2R*attitude[0]);
	cos_pitch = cos(D2R*attitude[1]);
	sin_pitch = sin(D2R*attitude[1]);
	// Tilt compensated magnetic field X component:
	headX = mag[0]*cos_pitch+mag[1]*sin_roll*sin_pitch+mag[2]*cos_roll*sin_pitch;
	// Tilt compensated magnetic field Y component:
	headY = mag[1]*cos_roll-mag[2]*sin_roll;
	// magnetic heading
	acc_attitude[2] = R2D*atan2(headY,-headX); //  + *MISC_DEC;  // heading with magnetic declination compensation (disabled)

	//	if(acc_attitude[2] > 180) acc_attitude[2] -=360;			// disabled because declination compesation is disabled
	//	else if(acc_attitude[2] <=-180) acc_attitude[2] +=360;

	euler2quat(qa, acc_attitude[0], acc_attitude[1], acc_attitude[2]); // Quaternions
}

void earthaccel(){

	float a[4];

	static float dt =0.01;
	static uint32_t lastus = 0, nowus=0;

	nowus = (LPC_TIM2->TC);

	if(nowus<lastus){

		dt = ((20000000 + nowus)-lastus)/1000000.0;  // 20s loop timer
	}
	else dt =  (nowus-lastus)/1000000.0;

	lastus = nowus;


	a[0] = 0; a[1] = acc[1]; a[2] = -acc[0]; a[3] = -acc[2]; // Quatenions -vectors

	//	norm = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3]);

	//	for(i=0; i<4; i++)a[i] = a[i]/norm;
	//
	//	euler2quat(q1, attitude[0], attitude[1], 0);
	//
	//	conj_quat(qj, q1);
	//	mult_quat(Qtemp, a, q1);
	//	mult_quat(a,qj,Qtemp);

	//	for(i=0; i<4; i++)a[i] = a[i]*norm;

	//	a[3] = -a[3] - 9.81;



	a[3] = sin(D2R*attitude[1])*acc[0] + sin(D2R*attitude[0])*cos(D2R*attitude[1])*acc[1] + cos(D2R*attitude[1])*cos(D2R*attitude[0])*acc[2];

	a[3] = a[3] - 9.81;

	if(a[3]>ACCDEADBAND){
		accvel = accvel + (a[3]-ACCDEADBAND)*dt; // velocity in m/s
		accvel = constrain(accvel, -3, 3);
	}
	else if(a[3]<-ACCDEADBAND){
		accvel = accvel + (a[3]+ACCDEADBAND)*dt;
		accvel = constrain(accvel, -3, 3);
	}



}



void earthaccel_1(){

	float q1[4];

	static float dt =0.01;
	static uint32_t lastus = 0, nowus=0;
	float cosphi, costheta, sinphi, sintheta, cosxi, sinxi;
	float K;
	float accr[3];

	float R[3][3];


	nowus = (LPC_TIM2->TC);

	if(nowus<lastus){

		dt = ((20000000 + nowus)-lastus)/1000000.0;  // 20s loop timer
	}
	else dt =  (nowus-lastus)/1000000.0;

	lastus = nowus;


//	euler2quat(&q1, attitude[0], attitude[1],OptitrackHeading);



//	qw=q1[0];
//	qx=q1[1];
//	qy=q1[2];
//	qz=q1[3];
//
//	//Rotation Matrix
//
//
//	R[0][0] = 	1 - 2*qy*qy - 2*qz*qz; 	R[0][1] = 2*qx*qy - 2*qz*qw;	R[0][2] = 2*qx*qz + 2*qy*qw;
//	R[1][0] = 	2*qx*qy + 2*qz*qw;	R[1][1] = 1 - 2*qx*qx - 2*qz*qz;	R[1][2] = 2*qy*qz - 2*qx*qw;
//	R[2][0] = 	2*qx*qz - 2*qy*qw;	R[2][1] = 2*qy*qz + 2*qx*qw;	R[2][2] = 1 - 2*qx*qx - 2*qy*qy;



//	AX_E = R[0][0]*acc[0] + R[0][1]*acc[1] + R[0][2]*acc[2];
//	AY_E = R[1][0]*acc[0] + R[1][1]*acc[1] + R[1][2]*acc[2];
//	AZ_E = R[2][0]*acc[0] + R[2][1]*acc[1] + R[2][2]*acc[2];


	costheta = cos(-D2R*attitude[1]); cosphi = cos(D2R*attitude[0]); cosxi = cos(D2R*(OptitrackHeading));
	sintheta = sin(-D2R*attitude[1]); sinphi = sin(D2R*attitude[0]); sinxi = sin(D2R*(OptitrackHeading));

	accr[0] = acc[0];

	accr[1] = acc[1]; // body frame acceleration
	accr[2] = acc[2];


	//  website link for the x y z combined rotation matrix //below are acceleration along earth frame
	AX_E = -(cosxi*costheta*accr[0] + (cosxi*sinphi*sintheta - cosphi*sinxi)*accr[1] + (sinphi*sinxi + cosphi*cosxi*sintheta)*accr[2]);

	AY_E = (costheta*sinxi*accr[0] + (cosphi*cosxi + sinphi*sinxi*sintheta)*accr[1] + (cosphi*sinxi*sintheta - cosxi*sinphi)*accr[2]);

	AZ_E = sin(D2R*attitude[1])*accr[0] + sin(D2R*attitude[0])*cos(D2R*attitude[1])*accr[1] + cos(D2R*attitude[1])*cos(D2R*attitude[0])*accr[2];


	AZ_E = AZ_E - 9.81;


	K = *MISC_MISSION_SIZE;

	if(AX_E>ACCDEADBAND){
		INS.Vx = INS.Vx + (AX_E - ACCDEADBAND)*dt;
	}
	else if(AX_E<-ACCDEADBAND){
		INS.Vx = INS.Vx + (AX_E + ACCDEADBAND)*dt;
	}


	INS.Vx = constrain(INS.Vx, -3, 3);


	INS.X = INS.X + INS.Vx*dt;


////////


	if(AY_E>ACCDEADBAND){
		INS.Vy = INS.Vy + (AY_E - ACCDEADBAND)*dt;
	}
	else if(AY_E<-ACCDEADBAND){
		INS.Vy = INS.Vy + (AY_E + ACCDEADBAND)*dt;
	}


	INS.Vy = constrain(INS.Vy, -3, 3);

	INS.Y = INS.Y + INS.Vy*dt;


////////
	if(AZ_E>ACCDEADBAND){
		INS.Vz = INS.Vz + (AZ_E - ACCDEADBAND)*dt;
	}
	else if(AZ_E<-ACCDEADBAND){
		INS.Vz = INS.Vz + (AZ_E + ACCDEADBAND)*dt;
	}


	INS.Vz = constrain(INS.Vz, -3, 3);

	INS.Z = INS.Z + INS.Vz*dt;


	nav.X = INS.X;
	nav.Y = INS.Y;
	nav.Z = INS.Z;
	nav.vx = INS.Vx;
	nav.vy = INS.Vy;
	nav.vz = INS.Vz;








	//
	//	if(a[3]>ACCDEADBAND){
	//		accvel = accvel + (a[3]-ACCDEADBAND)*dt; // velocity in m/s along z
	//		accvel = constrain(accvel, -3, 3);
	//	}
	//	else if(a[3]<-ACCDEADBAND){
	//		accvel = accvel + (a[3]+ACCDEADBAND)*dt;
	//		accvel = constrain(accvel, -3, 3);
	//	}
	//
	//	if(a[2]>ACCDEADBAND){
	//			accvel2 = accvel2 + (a[2]-ACCDEADBAND)*dt; // velocity in m/s along x
	//			accvel2 = constrain(accvel2, -3, 3);
	//		}
	//		else if(a[2]<-ACCDEADBAND){
	//			accvel2 = accvel + (a[2]+ACCDEADBAND)*dt;
	//			accvel2 = constrain(accvel2, -3, 3);
	//		}
	//
	//	if(a[1]>ACCDEADBAND){
	//			accvel1 = accvel1 + (a[1]-ACCDEADBAND)*dt; // velocity in m/s along y
	//			accvel1 = constrain(accvel1, -3, 3);
	//		}
	//		else if(a[1]<-ACCDEADBAND){
	//			accvel1 = accvel1 + (a[1]+ACCDEADBAND)*dt;
	//			accvel1 = constrain(accvel1, -3, 3);
	//		}








}

