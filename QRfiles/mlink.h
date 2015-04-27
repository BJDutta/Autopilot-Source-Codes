#ifndef MLINK_H
#define MLINK_H

#include "comm.h"
#include "bridge.h"
#include <mavlink/include/common/common.h>
#include <mavlink.h>
#include <mavlink_types.h>


#define ONBOARD_PARAM_COUNT 55
#define ONBOARD_PARAM_NAME_LENGTH 14



typedef struct
{
	union {
		float param[ONBOARD_PARAM_COUNT];
		char eeprom[ONBOARD_PARAM_COUNT*4];
	};
	char param_name[ONBOARD_PARAM_COUNT][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
}global_struct;



void mavlink_init();
void send_heartbeat();
void communication_receive();
void communication_queued_send();

extern int mode;

extern mavlink_sys_status_t mavlink_status;

extern uint16_t m_parameter_i;


extern global_struct global_data;

extern float *Kp, *Ki, *Kd;		// PID
extern float *Kpy, *Kiy, *Kdy;
extern float *Kpa, *Kia, *Kda;
extern float *Kpn, *Kin, *Kdn;

extern float *IMU_GAIN, *IMU_HZ, *IMU_RATIO, *ACCWEIGHT;
extern float *AccoffX, *AccoffY, *AccoffZ;
extern float *GyrooffX, *GyrooffY, *GyrooffZ;
extern float *MagoffX, *MagoffY, *MagoffZ;


float *RC_SENSE, *RC_YSENSE, *RC_ASENSE, *RC_GSENSE;
extern float *MOTOR_MIN, *MOTOR_MAX;
extern float *RC_PARAM_IDXL, *RC_PARAM_MINL, *RC_PARAM_MAXL, *RC_PARAM_IDXR, *RC_PARAM_MINR, *RC_PARAM_MAXR;


extern float *ILIMIT_RP, *ILIMIT_Y, *ILIMIT_ALT, *ILIMIT_XY, *ILIMIT_TILT;

// 42 parameters till here

extern float *MISC_DEC, *MISC_HOVALT, *MISC_MISSION_SIZE, *MISC_NAVALT;

extern float *FIELD_GEN, *FIELD_GRIDX, *FIELD_GRIDY, *FIELD_LENGTH, *FIELD_BREADTH, *FIELD_ALT, *FIELD_WPRADIUS, *FIELD_WPTIME;

extern float *HW_PROGRAM;

extern int firmware_flash;

#endif
