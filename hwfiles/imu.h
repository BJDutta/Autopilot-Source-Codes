#ifndef IMU_H
#define IMU_H

#include "MPU6050.h"
#include "HMC5883L.h"
#include "LED_Blink.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_uart.h"
#include "math.h"
#include "quat.h"
#include "lpc_types.h"


//#define PEDESTAL_MOUNT_MAG_NEO6M

#define ONBOARD_MAG




typedef struct {

	float roll;	// degrees -180 to +180
	float pitch; // degrees -90 to +90
	float yaw;	// degrees -180 to +180

	float rollrate; // degrees/s
	float pitchrate; // degrees/s
	float yawrate; // degrees/s

}imu_t;


extern imu_t imu;
extern imu_t cstarget;
extern imu_t Commtarget;


//#define GRAVITY 4096

#define GYRO_SENSITIVITY  16.4		// 16.4 LSB/deg/s  1 LSB = 1/16.4 deg/sec for full scale +- 2000 deg/s
#define MAG_SENSITIVITY  0.92


extern float GRAVITY;

//extern int accel_off[3], gyro_off[3], mag_off[3];

extern int32_t *ACC_OFF_X,*ACC_OFF_Y,*ACC_OFF_Z;
extern int32_t *GYR_OFF_X,*GYR_OFF_Y,*GYR_OFF_Z;
extern int32_t *MAG_OFF_X,*MAG_OFF_Y,*MAG_OFF_Z;

extern int16_t accel_imu[3];
extern int16_t gyro_imu[3];
extern int16_t mag_imu[3];
extern float acc_old[3];
extern int16_t x_accel_raw, y_accel_raw, z_accel_raw;

void imu_setup();
void imu_update();
void attitude_init();
void attitude_update();
float calculate_variance(uint8_t axis);
void read_sensors();
void read_gyros();
void calibrate_imu();
void gyro_calibrate();
void mag_calibrate();

#endif
