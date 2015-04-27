#ifndef QUAT_H
#define QUAT_H

#include "math.h"
#include "LED_Blink.h"
#include "stdlib.h"
#include "param.h"
#include "lpc_types.h"
#include "imu.h"



#define D2R 0.0174532925
#define R2D 57.2957795

#define ACCDEADBAND 0.3  // 1 m/s2

extern float *IMU_GAIN;

extern float gyro[3];
extern float acc[3];
extern float mag[3];
extern float q[4];
extern float qa[4];

extern float roll,pitch,yaw;
extern float a_roll,a_pitch,a_yaw;

extern float attitude[3];
extern float acc_attitude[3];

extern float accvel;
extern float accvel1;
extern float accvel2;

typedef struct{

	float  X;
	float  Y;
	float  Z;
	float  	Vx;
	float 	Vy;
	float 	Vz;

}ins_t;

extern ins_t INS;

extern float AX_E, AY_E, AZ_E;



float invSqrt(float number);
void init_quat();
float constrain(float var, float llimit, float ulimit );
void mult_quat(float *ab, float *a, float *b);
void conj_quat(float *qj, float *q);
void unit_quat(float *q);
float norm_quat(float *q);
void quat2angle(float *r, float *q);
void angle2quat(float *q, float *r);
void quat2euler(float *angles, float *q);
void euler2quat(float *q, float roll, float pitch, float yaw);
void integrate_gyro(float dt, char mergequat);
void merge_quats(float *q, float *qe, float *qa, float gain);
void acc_angles();
void earthaccel();
void earthaccel_1();


#endif
