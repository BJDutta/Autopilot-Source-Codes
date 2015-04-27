#include "imu.h"
#include "config.h"
#include "lpc17xx_wdt.h"
#include "param.h"
#include "i2ceeprom.h"
#include "mlink.h"
#include "lpc17xx_timer.h"
#include "quat.h"
#include "accelerometer_calibration.h"





imu_t imu;
imu_t cstarget;
imu_t Commtarget;
#define SAMPLE_SIZE  100

//float GRAVITY = 4096;

int16_t accel_imu[3];
int16_t gyro_imu[3];
int16_t mag_imu[3];

int16_t x_accel_raw;	//dummy variables for accelerometer calibration tests
int16_t y_accel_raw;
int16_t z_accel_raw;

float accel_scale[3];

float acc_old[3];

int16_t accel_off[3], gyro_off[3], mag_off[3];

float accel_off_f[3];
float accel_imu_f[3];


//int16_t samples[SAMPLE_SIZE][3];  // sample set for accelerometer variance


void detectslaves(){

	I2C_M_SETUP_Type transferMCfg;
	uint8_t wbuf[1];
	Status success;
	//	int16_t slaves[10];
	//	uint8_t index=0;
	uint8_t slaveAddr;
	//	uint8_t i=0;

	//	for(i=0; i<10; i++)slaves[i]=0;

	wbuf[0] = 0;
	//		wbuf[1] = pBuffer[0];

	for(slaveAddr=1; slaveAddr<250; slaveAddr++){
		/* Start I2C slave device first */
		transferMCfg.sl_addr7bit = slaveAddr;
		transferMCfg.tx_data = wbuf;
		transferMCfg.tx_length = 1;
		transferMCfg.rx_data = NULL;
		transferMCfg.rx_length = 0;
		transferMCfg.retransmissions_max = 5;
		success = I2C_MasterTransferData(MPU6050_I2C, &transferMCfg, I2C_TRANSFER_POLLING);

		if(success== SUCCESS) {
			//			slaves[index++]=slaveAddr;
			blink();
		}
	}

	//	blink();
}


void imu_setup()
{
	uint8_t tmp = 0x00;
	int idx, i;
	blink();
	MPU6050_I2C_Init();
	WDT_Feed();
	delay(100);
	MPU6050_Initialize();
	delay(100);
//	MPU6050_Aux_Bypass_Enable();		// disabling magnetometer here
//	delay(100);
//	HMC5883L_Initialize();
//	delay(100);
//	HMC5883L_I2C_ByteWrite(HMC5883L_DEFAULT_ADDRESS, &tmp,0x02);  // write 0x00 at 0x02 address to set in continuous conversion mode
//	delay(100);
//	MPU6050_ConfigMag();
//	delay(100);
									// disabling magnetometer till here

	//read_flash_imu();


	accel_off_f[0] = (*AccoffX); accel_off_f[1] = (*AccoffY); accel_off_f[2] = (*AccoffZ);		// restoring sensor offsets from eeprom parameters
	gyro_off[0] = (int16_t)(*GyrooffX); gyro_off[1] = (int16_t)(*GyrooffY); gyro_off[2] = (int16_t)(*GyrooffZ);
	accel_scale[0] = (*MagoffX); accel_scale[1] = (*MagoffY); accel_scale[2] = (*MagoffZ);

//	GRAVITY = sqrt((*AccoffX)*(*AccoffX) + (*AccoffY)*(*AccoffY) + (*AccoffZ)*(*AccoffZ));

//	GRAVITY = *AccoffZ;

	read_sensors();
	init_quat();

	//	detectslaves();
}

void imu_update()
{
	static float dt =0.01;
	static uint32_t lastus = 0, nowus=0;
	static uint16_t gyronum = 0;

	nowus = (LPC_TIM2->TC);

	if(nowus<lastus){

		dt = ((20000000 + nowus)-lastus)/1000000.0;  // 20s loop timer
	}
	else dt =  (nowus-lastus)/1000000.0;

	lastus = nowus;

//	if(++gyronum>=*IMU_RATIO){				// different rates of gyro & acc removed, acc+gyro merge in every cycle (~200Hz)
//		read_sensors();
//		acc_angles();
//		integrate_gyro(dt,1);
//
//		earthaccel();
//
//
//		gyronum=0;
//		//		blink();
//	}
//	else{
//		read_gyros();
//		integrate_gyro(dt, 0);
//
//	}

	read_sensors();
	//acc_angles();
	integrate_gyro(dt,1);
	quat2euler(attitude, q);
	earthaccel_1();



	imu.roll = attitude[0];
	imu.pitch = attitude[1];
	imu.yaw = attitude[2];
	imu.rollrate = R2D*gyro[0];
	imu.pitchrate = R2D*gyro[1];
	imu.yawrate = R2D*gyro[2];
}

float calculate_variance(uint8_t axis){

//	int  j;
//	float mean=0, sigma=0;
//
//	for(j=0; j<SAMPLE_SIZE; j++)mean+=samples[j][axis];
//
//	mean/=SAMPLE_SIZE;
//
//	for(j=0; j<SAMPLE_SIZE; j++)sigma+=((samples[j][axis]-mean)*(samples[j][axis]-mean));
//
//	sigma/=SAMPLE_SIZE;
//
//	return sigma;
}

void read_sensors()
{
	int i=0, success;
	//	static int idx=0;
	//
	//	float sigma=0;


	success = MPU6050_GetRawAccelGyroMag(accel_imu,gyro_imu,mag_imu);

		x_accel_raw=accel_imu[0];
		y_accel_raw=accel_imu[1];
		z_accel_raw=accel_imu[2];

	if(success==0)
	{
		MPU6050_I2C_Reboot();
		return;
	}
	for(i=0;i<3;i++)
	{
		accel_imu_f[i] = (float)accel_imu[i]-accel_off_f[i];
		gyro_imu[i]-=gyro_off[i];
		mag_imu[i]-=mag_off[i];
	}

	//	accel_imu[0]-= (*ACC_OFF_X);
	//	accel_imu[1]-= (*ACC_OFF_Y);
	//	accel_imu[2]-= (*ACC_OFF_Z);
	//
	//	gyro_imu[0]-=(*GYR_OFF_X);
	//	gyro_imu[1]-=(*GYR_OFF_Y);
	//	gyro_imu[2]-=(*GYR_OFF_Z);

	accel_imu_f[0] = -accel_imu_f[0];
	accel_imu_f[1] = -accel_imu_f[1];
//	accel_imu_f[2] += GRAVITY;
	// reverse signs
	gyro_imu[0]= -gyro_imu[0];
	gyro_imu[2]= -gyro_imu[2];

#ifdef PEDESTAL_MOUNT_MAG_NEO7M
	{
		int16_t temp=0;
		temp = mag_imu[0];
		mag_imu[0] = mag_imu[1];		// swap X,Y , inverse Z
		mag_imu[1] = temp;
		mag_imu[2] = -mag_imu[2];

		mag_imu[1] = -mag_imu[1];		// inverse Y for previous directions

	}
#else
#ifdef PEDESTAL_MOUNT_MAG_NEO6M
	int16_t temp=0;
			temp = mag_imu[0];
			mag_imu[0] = mag_imu[1];		// swap X,Y , inverse Z
			mag_imu[1] = temp;
			mag_imu[2] = mag_imu[2];

			mag_imu[1] = mag_imu[1];		// inverse Y for previous directions
			mag_imu[0] = -mag_imu[0];


#else
	mag_imu[0] = -mag_imu[0];
	mag_imu[1] = mag_imu[1];
	mag_imu[2] = -mag_imu[2];
#endif
#endif

	//	for(i=0;i<3;i++){
	//		samples[idx][i] = accel_imu[i];
	//		sigma = sigma + calculate_variance(i);
	//	}
	//	if(++idx==SAMPLE_SIZE)idx=0;

	for(i=0;i<3;i++)
	{
		acc[i] = acc_old[i]*(1-*ACCWEIGHT) + accel_imu_f[i]*accel_scale[i]*(*ACCWEIGHT);
		acc_old[i]=acc[i];
		gyro[i]=D2R*gyro_imu[i]/GYRO_SENSITIVITY;
		mag[i]=mag_imu[i]*MAG_SENSITIVITY;
	}

	// compensate centrifugal accelerations
	//	if(ublox.fix==1 && ublox.gps_speed>1)
	//	{
	////		crabangle =
	//		acc[1] = acc[1] + ublox.gps_speed*gyro[2];//*cos(d2r*(gpsheading-yaw));
	//		acc[2] = acc[2] - ublox.gps_speed*gyro[1];
	//		acc[0] = acc[0] - ublox.gps_acc;
	//	}
}

void read_gyros(){

	int i=0, success;
	success = MPU6050_GetRawGyro(gyro_imu);
	if(success==0)
	{
		MPU6050_I2C_Reboot();
		return;
	}
	for(i=0;i<3;i++)
	{
		gyro_imu[i]-=gyro_off[i];
	}


	// reverse signs
	gyro_imu[0]= -gyro_imu[0];
	gyro_imu[2]= -gyro_imu[2];


	for(i=0;i<3;i++)
	{
		gyro[i]=D2R*gyro_imu[i]/GYRO_SENSITIVITY;
	}

}

void calibrate_imu()
{

	do_accel_calibration();




//	int i=0,j=0;
//
//	int32_t accel_off1[3], gyro_off1[3];
//
//
//	delay(1000);
//	for(i=0; i<70; i++)
//	{
//		WDT_Feed();
//		delay(80);
//		blink();
//	}
//	for(i=0;i<3;i++)
//	{
//		accel_off1[i] = 0;
//		gyro_off1[i]=0;
//	}
//
//	//	(*ACC_OFF_X) = 0;
//	//	(*ACC_OFF_Y) = 0;
//	//	(*ACC_OFF_Z) = 0;
//	//
//	//	(*GYR_OFF_X) = 0;
//	//	(*GYR_OFF_Y) = 0;
//	//	(*GYR_OFF_Z) = 0;
//	//
//	//	(*MAG_OFF_X) = 0;
//	//	(*MAG_OFF_Y) = 0;
//	//	(*MAG_OFF_Z) = 0;
//
//	for(j=0; j<50; j++)
//	{
//		delay(100);
//		WDT_Feed();
//		MPU6050_GetRawAccelGyroMag(accel_imu,gyro_imu,mag_imu);
//		for(i=0;i<3;i++)
//		{
//			accel_off1[i]+=accel_imu[i];
//			gyro_off1[i]+=gyro_imu[i];
//
//		}
//
//		//		(*ACC_OFF_X) +=accel_imu[0];
//		//		(*ACC_OFF_Y) +=accel_imu[1];
//		//		(*ACC_OFF_Z) +=accel_imu[2];
//		//
//		//		(*GYR_OFF_X) +=gyro_imu[0];
//		//		(*GYR_OFF_Y) +=gyro_imu[1];
//		//		(*GYR_OFF_Z) +=gyro_imu[2];
//		//
//		//		(*MAG_OFF_X) +=mag_imu[0];
//		//		(*MAG_OFF_Y) +=mag_imu[1];
//		//		(*MAG_OFF_Z) +=mag_imu[2];
//
//		//		IWDG_ReloadCounter();
//	}
//
//	for(i=0;i<3;i++)
//	{
//		accel_off1[i]/=50;
//		gyro_off1[i]/=50;
//
//
//		accel_off[i] = (int16_t)accel_off1[i];
//		gyro_off[i] = (int16_t)gyro_off1[i];
//
//	}
//	//	(*ACC_OFF_X)/=50;
//	//	(*ACC_OFF_Y)/=50;
//	//	(*ACC_OFF_Z)/=50;
//	//	(*GYR_OFF_X) /=50;
//	//	(*GYR_OFF_Y) /=50;
//	//	(*GYR_OFF_Z) /=50;
//	//	(*MAG_OFF_X) /=50;
//	//	(*MAG_OFF_Y) /=50;
//	//	(*MAG_OFF_Z) /=50;



	init_quat();
}

void gyro_calibrate(){
	int i=0,j=0;

	int32_t gyro_off1[3];


	for(i=0; i<12; i++)
	{
		WDT_Feed();
		delay(50);
		blink();
	}
	for(i=0;i<3;i++)
	{
		gyro_off1[i]=0;
	}

	for(j=0; j<20; j++)
	{
		delay(20);
		blink();
		WDT_Feed();

		MPU6050_GetRawGyro(gyro_imu);

		for(i=0;i<3;i++)
		{
			gyro_off1[i]+=gyro_imu[i];

		}
	}

	for(i=0;i<3;i++)
	{
		gyro_off1[i]/=20;

		gyro_off[i] = (int16_t)gyro_off1[i];
	}


	*GyrooffX = (float)gyro_off[0]; *GyrooffY = (float)gyro_off[1]; *GyrooffZ = (float)gyro_off[2]; //offset along the three axis

	eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));

}

void mag_calibrate(){

//	int32_t magZeroTempMin[3];
//	int32_t magZeroTempMax[3];
//	uint8_t axis, i;
//
//	uint32_t lastus = 0, nowus=0;
//
//
//	delay(100); blink();
//	WDT_Feed();
//	MPU6050_GetRawAccelGyroMag(accel_imu,gyro_imu,mag_imu);
//
//
//	for(axis=0;axis<3;axis++) {
//		mag_off[axis] = 0;
//		magZeroTempMin[axis] = mag_imu[axis];
//		magZeroTempMax[axis] = mag_imu[axis];
//	}
//
//	for(i=0; i<2; i++){
//
//		lastus = nowus = (LPC_TIM2->TC);
//
//		while((nowus - lastus) < 15000000) { // 15s * 2: you have 30s to turn the UAV in all directions
//
//			nowus = (LPC_TIM2->TC);
//
//			if(nowus<lastus)nowus +=20000000;
//
//
//			delay(25); blink();
//			WDT_Feed();
//			MPU6050_GetRawAccelGyroMag(accel_imu,gyro_imu,mag_imu);
//
//			for(axis=0;axis<3;axis++) {
//				if (mag_imu[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = mag_imu[axis];
//				if (mag_imu[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = mag_imu[axis];
//			}
//		}
//	}
//	for(axis=0;axis<3;axis++)mag_off[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
//
//
//	*MagoffX = (float)mag_off[0]; *MagoffY = (float)mag_off[1]; *MagoffZ = (float)mag_off[2];
//
//	eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));
}
