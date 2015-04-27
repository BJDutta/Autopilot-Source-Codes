#include "mlink.h"
#include "config.h"
#include <math.h>

#include "lpc_types.h"

#include "LED_Blink.h"
#include "mathfunc.h"
#include "imu.h"
#include "control.h"
#include "param.h"
#include "i2ceeprom.h"
#include "waypoint.h"
#include "timing.h"
#include "gps.h"

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
/* The default UART header for your MCU */



mavlink_sys_status_t mavlink_status;

static int packet_drops = 0;
int mode = 0; /* Defined in mavlink_types.h, which is included by mavlink.h */


global_struct global_data;

float *Kp, *Ki, *Kd;		// PID
float *Kpy, *Kiy, *Kdy;
float *Kpa, *Kia, *Kda;
float *Kpn, *Kin, *Kdn;

float *IMU_GAIN, *IMU_HZ, *IMU_RATIO, *ACCWEIGHT;
float *AccoffX, *AccoffY, *AccoffZ;
float *GyrooffX, *GyrooffY, *GyrooffZ;
float *MagoffX, *MagoffY, *MagoffZ;


float *RC_SENSE, *RC_YSENSE, *RC_ASENSE, *RC_GSENSE;
float *MOTOR_MIN, *MOTOR_MAX;
float *RC_PARAM_IDXL, *RC_PARAM_MINL, *RC_PARAM_MAXL, *RC_PARAM_IDXR, *RC_PARAM_MINR, *RC_PARAM_MAXR;


float *ILIMIT_RP, *ILIMIT_Y, *ILIMIT_ALT, *ILIMIT_XY, *ILIMIT_TILT;


float *MISC_DEC, *MISC_HOVALT, *MISC_MISSION_SIZE, *MISC_NAVALT;

float *FIELD_GEN, *FIELD_GRIDX, *FIELD_GRIDY, *FIELD_LENGTH, *FIELD_BREADTH, *FIELD_ALT, *FIELD_WPRADIUS, *FIELD_WPTIME;

float *HW_PROGRAM;

int firmware_flash = 0;


/**
 * @brief reset all parameters to default
 * @warning DO NOT USE THIS IN FLIGHT!
 */
static inline void global_data_reset_param_defaults(void)
{
	uint8_t index =0;
	//	global_data.param[PARAM_SYSTEM_ID] = 13;
	//	strcpy(global_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");
	//
	//	// attitude
	//	global_data.param[PARAM_COMPONENT_ID] = 50;
	//	strcpy(global_data.param_name[PARAM_COMPONENT_ID], "IMU_ID");

	//	global_data.param[PARAM_PID_P] = .002;
	//	strcpy(global_data.param_name[PARAM_PID_P], "PID_P");
	//	Kp = &global_data.param[PARAM_PID_P];
	//
	//	global_data.param[PARAM_PID_I] = 0;
	//	strcpy(global_data.param_name[PARAM_PID_I], "PID_I");
	//
	//
	//	global_data.param[PARAM_PID_D] = .0005;
	//	strcpy(global_data.param_name[PARAM_PID_D], "PID_D");
	//	Kd = &global_data.param[PARAM_PID_D];
	//
	//	//heading control
	//	global_data.param[PARAM_PID_PY] = .0001;
	//	strcpy(global_data.param_name[PARAM_PID_PY], "PID_PY");
	//	Kpy = &global_data.param[PARAM_PID_PY];
	//
	//	global_data.param[PARAM_PID_IY] = 0;
	//	strcpy(global_data.param_name[PARAM_PID_IY], "PID_IY");
	//
	//	global_data.param[PARAM_PID_DY] = .0001;
	//	strcpy(global_data.param_name[PARAM_PID_DY], "PID_DY");
	//	Kdy = &global_data.param[PARAM_PID_DY];
	//
	//	//altitude control
	//	global_data.param[PARAM_PID_PA] = .0001;
	//	strcpy(global_data.param_name[PARAM_PID_PA], "PID_PA");
	//	Kpa = &global_data.param[PARAM_PID_PA];
	//
	//	global_data.param[PARAM_PID_IA] = 0.001;
	//	strcpy(global_data.param_name[PARAM_PID_IA], "PID_IA");
	//	Kia = &global_data.param[PARAM_PID_IA];
	//
	//	global_data.param[PARAM_PID_DA] = .0001;
	//	strcpy(global_data.param_name[PARAM_PID_DA], "PID_DA");
	//	Kda = &global_data.param[PARAM_PID_DA];

	global_data.param[index] = .002; strcpy(global_data.param_name[index], "PID_P");
	Kp = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "PID_I");
	Ki = &global_data.param[index++];


	global_data.param[index] = .0005; strcpy(global_data.param_name[index], "PID_D");
	Kd = &global_data.param[index++];

	//heading control
	global_data.param[index] = .0001; strcpy(global_data.param_name[index], "PID_PY");
	Kpy = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "PID_IY");
	Kiy = &global_data.param[index++];

	global_data.param[index] = .0001; strcpy(global_data.param_name[index], "PID_DY");
	Kdy = &global_data.param[index++];

	//altitude control
	global_data.param[index] = .0001; strcpy(global_data.param_name[index], "PID_PA");
	Kpa = &global_data.param[index++];

	global_data.param[index] = 0.001; strcpy(global_data.param_name[index], "PID_IA");
	Kia = &global_data.param[index++];

	global_data.param[index] = .0001; strcpy(global_data.param_name[index], "PID_DA");
	Kda = &global_data.param[index++];

	//navigation control
	global_data.param[index] = .0001; strcpy(global_data.param_name[index], "PID_PN");
	Kpn = &global_data.param[index++];

	global_data.param[index] = 0.001; strcpy(global_data.param_name[index], "PID_IN");
	Kin = &global_data.param[index++];

	global_data.param[index] = .0001; strcpy(global_data.param_name[index], "PID_DN");
	Kdn = &global_data.param[index++];


	// if PID parameters appended, update number in POT CTRL of parameters

	// IMU parameters

	global_data.param[index] = 0.2; strcpy(global_data.param_name[index], "IMU_GAIN");
	IMU_GAIN = &global_data.param[index++];

	global_data.param[index] = 100; strcpy(global_data.param_name[index], "IMU_HZ");
	IMU_HZ = &global_data.param[index++];

	global_data.param[index] = 2; strcpy(global_data.param_name[index], "IMU_RATIO");
	IMU_RATIO = &global_data.param[index++];

	global_data.param[index] = 0.2; strcpy(global_data.param_name[index], "IMU_ACCWEIGHT");
	ACCWEIGHT = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_ACCX");
	AccoffX = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_ACCY");
	AccoffY = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_ACCZ");
	AccoffZ = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_GYROX");
	GyrooffX = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_GYROY");
	GyrooffY = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_GYROZ");
	GyrooffZ = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_MAGX");
	MagoffX = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_MAGY");
	MagoffY = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "IMU_MAGZ");
	MagoffZ = &global_data.param[index++];


	// RC or sticks or motors params

	global_data.param[index] = 20; strcpy(global_data.param_name[index], "RC_SENSE");
	RC_SENSE = &global_data.param[index++];

	global_data.param[index] = 20; strcpy(global_data.param_name[index], "RC_YSENSE");
	RC_YSENSE = &global_data.param[index++];

	global_data.param[index] = 20; strcpy(global_data.param_name[index], "RC_ASENSE");
	RC_ASENSE = &global_data.param[index++];

	global_data.param[index] = 20; strcpy(global_data.param_name[index], "RC_GSENSE");
	RC_GSENSE = &global_data.param[index++];

	global_data.param[index] = 1000; strcpy(global_data.param_name[index], "RC_MOTOR_MIN");
	MOTOR_MIN = &global_data.param[index++];

	global_data.param[index] = 1900; strcpy(global_data.param_name[index], "RC_MOTOR_MAX");
	MOTOR_MAX = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "RC_PARAM_IDXL");
	RC_PARAM_IDXL = &global_data.param[index++];

	global_data.param[index] = 0.001; strcpy(global_data.param_name[index], "RC_PARAM_MINL");
	RC_PARAM_MINL = &global_data.param[index++];

	global_data.param[index] = 0.1; strcpy(global_data.param_name[index], "RC_PARAM_MAXL");
	RC_PARAM_MAXL = &global_data.param[index++];


	global_data.param[index] = 0; strcpy(global_data.param_name[index], "RC_PARAM_IDXR");
	RC_PARAM_IDXR = &global_data.param[index++];

	global_data.param[index] = 0.001; strcpy(global_data.param_name[index], "RC_PARAM_MINR");
	RC_PARAM_MINR = &global_data.param[index++];

	global_data.param[index] = 0.1; strcpy(global_data.param_name[index], "RC_PARAM_MAXR");
	RC_PARAM_MAXR = &global_data.param[index++];

	// ILIMITs

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "ILIMIT_RP");
	ILIMIT_RP = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "ILIMIT_Y");
	ILIMIT_Y = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "ILIMIT_ALT");
	ILIMIT_ALT = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "ILIMIT_XY");
	ILIMIT_XY = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "ILIMIT_TILT");
	ILIMIT_TILT = &global_data.param[index++];


	// Miscellaneous parameters

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "MISC_DEC");		// Magnetic Declination
	MISC_DEC = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "MISC_HOVALT");		// Altitude mission altitude
	MISC_HOVALT = &global_data.param[index++];

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "MISC_MISSIONSIZE");		// Magnetic calibration for 30 secs
	MISC_MISSION_SIZE = &global_data.param[index++];


	global_data.param[index] = 7; strcpy(global_data.param_name[index], "MISC_NAVALT");		// navigation altitude to home
	MISC_NAVALT = &global_data.param[index++];


	// field parameters

	global_data.param[index] = 0; strcpy(global_data.param_name[index], "FIELD_GEN");		// To generate navigation points in field
	FIELD_GEN = &global_data.param[index++];

	global_data.param[index] = 5; strcpy(global_data.param_name[index], "FIELD_GRIDX");
	FIELD_GRIDY = &global_data.param[index++];

	global_data.param[index] = 5; strcpy(global_data.param_name[index], "FIELD_GRIDY");
	FIELD_GRIDX = &global_data.param[index++];

	global_data.param[index] = 25; strcpy(global_data.param_name[index], "FIELD_LENGTH");
	FIELD_LENGTH   = &global_data.param[index++];

	global_data.param[index] = 25; strcpy(global_data.param_name[index], "FIELD_BREADTH");
	FIELD_BREADTH  = &global_data.param[index++];

	global_data.param[index] = 10; strcpy(global_data.param_name[index], "FIELD_ALT");
	FIELD_ALT  = &global_data.param[index++];

	global_data.param[index] = 10; strcpy(global_data.param_name[index], "FIELD_WPRADIUS");
	FIELD_WPRADIUS  = &global_data.param[index++];

	global_data.param[index] = 5; strcpy(global_data.param_name[index], "FIELD_WPTIME");
	FIELD_WPTIME  = &global_data.param[index++];


	global_data.param[index] = 0; strcpy(global_data.param_name[index], "HW_PROGRAM");
	HW_PROGRAM  = &global_data.param[index++];

	//			eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));




}

uint16_t m_parameter_i = 0;


void mavlink_init(){
	mavlink_system.sysid = 13; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

	mavlink_system.type = MAV_TYPE_QUADROTOR;
	mavlink_system.mode = MAV_MODE_MANUAL_DISARMED;
	mavlink_system.nav_mode = MAV_MODE_MANUAL_DISARMED;
	mavlink_system.state = MAV_STATE_ACTIVE;


	mavlink_status.battery_remaining=90;
	mavlink_status.load = 500;
	mavlink_status.voltage_battery = 11100;
	mavlink_status.current_battery = -1;
	//	malink_status.

	global_data_reset_param_defaults();

	eeprom_read(global_data.eeprom, sizeof(global_data.eeprom));

}


/**
 * @brief Receive communication packets and handle them
 *
 * This function decodes packets on the protocol level and also handles
 * their value by calling the appropriate functions.
 */
void communication_receive()
{
	mavlink_message_t msg;
	mavlink_status_t status;

	static unsigned char serBuf [RX_BUF_SIZE];
	int  numBytesRead;
	int index=0;
	uint16_t i=0, j=0;


	uint16_t wpnumber = 0;

	numBytesRead = tel_receive(&serBuf[0]);

	if(!numBytesRead) return;


	// COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)

	while(numBytesRead--)
	{
		uint8_t c = serBuf[index++];
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message


			switch(msg.msgid)
			{
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				// E.g. read GCS heartbeat and go into
				// comm lost mode if timer times out
								//			blink();
												GPS_blink();
			}
			break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
			{// EXECUTE ACTION
				mavlink_command_long_t command_long;

				mavlink_msg_command_long_decode(&msg, &command_long);
				blink();
				switch(command_long.command){

				case MAV_CMD_NAV_TAKEOFF:
				{
					if(RCmode.arm){
						CommMode.alt = TAKEOFF;
						CommMode.gps = 1;
						CommMode.nav = LOITER;
						tablet_armed = 0;
					}
				}
				break;
				case MAV_CMD_NAV_LAND:
				{
					if(RCmode.arm){		// landing at the same position
						CommMode.alt = LANDING;
						CommMode.gps = 1;
						CommMode.nav = LOITER;
						tablet_armed = 0;
					}
				}
				break;
				case MAV_CMD_NAV_RETURN_TO_LAUNCH:
				{
					if(RCmode.arm){		// landing at the same position
						CommMode.alt = ALT_HOLD;
						CommMode.gps = 1;
						CommMode.nav = RETURN_TO_HOME;
					}
				}
				break;
				case MAV_CMD_NAV_LOITER_UNLIM:
				{
					if(RCmode.arm){		// loiter at the same position
						CommMode.alt = ALT_HOLD;
						CommMode.gps = 1;
						CommMode.nav = LOITER;
					}
				}
				break;
				case MAV_CMD_OVERRIDE_GOTO:
				{
					targetnav.X = command_long.param5;
					targetnav.Y = command_long.param6;
					targetnav.Z = command_long.param7;

				}
				break;
				case MAV_CMD_COMPONENT_ARM_DISARM:{
					//					if(command_long.param1==1){
					//						CS_arm();
					//					}
					//					if(command_long.param1==0){
					//						CS_disarm();
					//					}
				}
				break;

				case MAV_CMD_CONDITION_YAW:
				{
					//					float yaw_target = command_long.param1;
					//					if(yaw_target<=360 && yaw_target>=-180){
					//						if(yaw_target>180)yaw_target -=360;
					//						if(yaw_target<=-180)yaw_target+=360;
					//						Commtarget.yaw = yaw_target;
					//					}
				}
				break;
				case MAV_CMD_CONDITION_CHANGE_ALT:
				{
					if(command_long.param7<100 && command_long.param7>-10){		// -10 to 100 meters of altitude range AGL
						targetnav.Z= command_long.param7;
					}

				}
				break;
				//				case MAV_CMD_DO_CHANGE_SPEED:
				//				{
				//					float speed = command_long.param2;
				//					if(speed<0)break;
				//					if(speed>MAX_GROUND_SPEED)speed = MAX_GROUND_SPEED;
				//					target_airspeed = speed;
				//					RCmode.nav = VELOCITY_CONTROL;
				//				}
				//				break;
				case MAV_CMD_PREFLIGHT_CALIBRATION:
				{
#ifdef HIL_MODE
					//Pop an error for hil mode/no calibration
#else
					if(command_long.param2==1) mag_calibrate();
					if(command_long.param5==1)calibrate_imu();  // calibrate imu sensors
#endif


				}
				break;
				case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				{
					//					if(command_long.param1==1){
					eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));
					NVIC_SystemReset();
					//					}
				}
				break;
				case MAV_CMD_PREFLIGHT_STORAGE:	// go to the ISP mode for firmware upgrade
				{
					firmware_flash=1;
				}
				break;

				case MAV_CMD_DO_CONTROL_VIDEO:
				{
					//					if(command_long.param6==1){
					//						isVideoTest = 1;
					//						trigger(10000);
					//						isVideoTest = 0;
					//					}
					//					else {
					//						isPhotoTest=1;
					//						trigger(1000);
					//						isPhotoTest=0;
					//					}
				}
				break;
				default: break;
				}
				mavlink_msg_heartbeat_send(MAVLINK_COMM_0, mavlink_system.type, MAV_AUTOPILOT_GENERIC, mavlink_system.mode, mavlink_system.nav_mode, mavlink_system.state); //tel_transmit();
				mavlink_msg_command_ack_send(MAVLINK_COMM_0, command_long.command, MAV_RESULT_ACCEPTED);  			tel_transmit();
			}
			break;
			case MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST:
			{
				//				blink();
				//				if(RCmode.arm){		// attitude control
				//					CommMode.alt = ALT_HOLD;
				//					CommMode.gps = 0;
				//					CommMode.nav = FREE;
				//
				//					Commtarget.roll = mavlink_msg_set_roll_pitch_yaw_thrust_get_roll(&msg);
				//					Commtarget.pitch = mavlink_msg_set_roll_pitch_yaw_thrust_get_pitch(&msg);
				//					Commtarget.yaw = mavlink_msg_set_roll_pitch_yaw_thrust_get_yaw(&msg);
				//
				//				}
			}
			break;
			case MAVLINK_MSG_ID_SET_MODE:
			{
				mavlink_system.mode = mavlink_msg_set_mode_get_base_mode(&msg);
				mode = mavlink_msg_set_mode_get_custom_mode(&msg);
				//				blink();

				mavlink_msg_command_ack_send(MAVLINK_COMM_0, MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED);  			tel_transmit();

			}
			break;
			case MAVLINK_MSG_ID_HIL_STATE:
			{
				imu.roll  = r2d*mavlink_msg_hil_state_get_roll(&msg);
				imu.pitch = r2d*mavlink_msg_hil_state_get_pitch(&msg);
				imu.yaw   = r2d*mavlink_msg_hil_state_get_yaw(&msg);
				if(imu.yaw>180)imu.yaw-=360;
				imu.rollrate = r2d*mavlink_msg_hil_state_get_pitchspeed(&msg);
				imu.pitchrate = r2d*mavlink_msg_hil_state_get_yawspeed(&msg);
				imu.yawrate = r2d*mavlink_msg_hil_state_get_rollspeed(&msg);

				nav.X = mavlink_msg_hil_state_get_lat(&msg);
				nav.Y = mavlink_msg_hil_state_get_lon(&msg);
				nav.Z = ((float)mavlink_msg_hil_state_get_alt(&msg)/1000.0) - homealt;

				//				blink();

			}
			break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			{
				// Start sending parameters
				m_parameter_i = 0;
				//				blink();
			}
			break;
			case MAVLINK_MSG_ID_PARAM_SET:
			{
				mavlink_param_set_t set;
				mavlink_msg_param_set_decode(&msg, &set);

				blink();
				// Check if this message is for this system
				if ((uint8_t) set.target_system
						== mavlink_system.sysid
						&& (uint8_t) set.target_component
						== mavlink_system.compid)
				{
					char* key = (char*) set.param_id;

					for (i = 0; i < ONBOARD_PARAM_COUNT; i++)
					{
						Bool match = TRUE;
						for (j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++)
						{
							// Compare
							if (((char) (global_data.param_name[i][j]))
									!= (char) (key[j]))
							{
								match = FALSE;
							}

							// End matching if null termination is reached
							if (((char) global_data.param_name[i][j]) == '\0')
							{
								break;
							}
						}

						// Check if matched
						if (match)
						{
							// Only write and emit changes if there is actually a difference
							// AND only write if new value is NOT "not-a-number"
							// AND is NOT infinity
							//							if (global_data.param[i] != set.param_value &&  // this line edited from mavlink to enable acknoledgement on overwriting the same value of the parameter
							if(		 !isnan(set.param_value)
									&& !isinf(set.param_value) && set.param_type == MAVLINK_TYPE_FLOAT)
							{
								global_data.param[i] = set.param_value;
								// Report back new value
								mavlink_msg_param_value_send(MAVLINK_COMM_0,
										(char*) global_data.param_name[i],
										global_data.param[i], MAVLINK_TYPE_FLOAT,
										ONBOARD_PARAM_COUNT, i);  			tel_transmit();

								eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));
							}
						}
					}
				}
			}
			break;

			case MAVLINK_MSG_ID_MANUAL_CONTROL:
			{
				rc.roll = mavlink_msg_manual_control_get_x(&msg);
				rc.pitch = mavlink_msg_manual_control_get_y(&msg);
				rc.throttle = mavlink_msg_manual_control_get_z(&msg);
				rc.yaw = mavlink_msg_manual_control_get_r(&msg);


				//				blink();
			}
			break;

			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			{
				blink();
				//mavlink_msg_global_position_int_send(MAVLINK_COMM_0, millisnow, nav.lat, nav.lon, (nav.alt + homealt)*1000, nav.alt*1000, GPS._vel_north, GPS._vel_east, nav.vz*100, GPS.ground_course)
				//   mavlink_global_position_int_t(3,x*1000,y*1000,z*1000,timelast,vx,vy,vz,yawlast);
				//Data parameters same as the sent by optitarck
				//OptitrackTracking = mavlink_msg_global_position_int_get_relative_alt(&msg)/1000.0f;
				OptitrackX = mavlink_msg_global_position_int_get_lat(&msg)/1000.0f;
				OptitrackY= mavlink_msg_global_position_int_get_lon(&msg)/1000.0f;
				OptitrackZ = mavlink_msg_global_position_int_get_alt(&msg)/1000.0f;
				OptitrackError = mavlink_msg_global_position_int_get_relative_alt(&msg)/100000.0f;
				OptitrackVx = mavlink_msg_global_position_int_get_vx(&msg)/100.0f;
				OptitrackVy = mavlink_msg_global_position_int_get_vy(&msg)/100.0f;
				OptitrackVz = mavlink_msg_global_position_int_get_vz(&msg)/100.0f;
				OptitrackHeading = mavlink_msg_global_position_int_get_hdg(&msg)/100.0f;
				if(0.0<OptitrackError && OptitrackError<1.0) OptitrackTracking = TRUE;
				else OptitrackTracking = FALSE;

				navigation();

				GPS_blink();
			}
			break;

			case MAVLINK_MSG_ID_MISSION_COUNT:
			{
				mavlink_mission_count_t mission_count;

				*MISC_MISSION_SIZE = mavlink_msg_mission_count_get_count(&msg);

				mavlink_msg_mission_count_decode(&msg, &mission_count);

				wpnumber = 0;
				mavlink_msg_mission_request_send(MAVLINK_COMM_0, msg.sysid, 50, wpnumber);  tel_transmit();
			}
			break;

			case MAVLINK_MSG_ID_MISSION_ITEM:
			{
				mavlink_mission_item_t temp;

				//				blink();

				wpnumber = mavlink_msg_mission_item_get_seq(&msg);
				mavlink_msg_mission_item_decode(&msg, &temp);
				WP.item[wpnumber] = temp;



				if((wpnumber+1)>=*MISC_MISSION_SIZE){
					//
					//					mavlink_msg_mission_request_send(MAVLINK_COMM_0,msg.sysid, msg.compid, wpnumber+1);  tel_transmit();
					mavlink_msg_mission_ack_send(MAVLINK_COMM_0, 255, 0, MAV_MISSION_ACCEPTED);  tel_transmit();

					waypoint_save();											// waypoint write to eeprom
					eeprom_write(global_data.eeprom, sizeof(global_data.eeprom)); // parameter write for mission size
				}
				else{
					mavlink_msg_mission_request_send(MAVLINK_COMM_0,msg.sysid, msg.compid, wpnumber+1);  tel_transmit();
				}


			}
			break;

			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
			{
				mavlink_mission_request_list_t mission_request_list;
				mavlink_msg_mission_request_list_decode(&msg, &mission_request_list);

				mavlink_msg_mission_count_send(MAVLINK_COMM_0, msg.sysid, mission_request_list.target_component , *MISC_MISSION_SIZE); tel_transmit();

				//				blink();
			}
			break;

			case MAVLINK_MSG_ID_MISSION_REQUEST:
			{
				mavlink_mission_item_t temp;

				temp =	WP.item[mavlink_msg_mission_request_get_seq(&msg)];
				mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg.sysid, temp.target_component,temp.seq, temp.frame, temp.command, temp.current, temp.autocontinue, temp.param1, temp.param2, temp.param3, temp.param4, temp.x, temp.y, temp.z);
				tel_transmit();

				//				blink();
			}
			break;

			case MAVLINK_MSG_ID_VICON_5Q:

			{
				//				blink();
			}


			default:
				//Do nothing
				//								imu.roll= msg.msgid;
				//				blink();
				break;
			}
		}

		// And get the next one
	}

	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;

}

/**
 * @brief Send low-priority messages at a maximum rate of xx Hertz
 *
 * This function sends messages at a lower rate to not exceed the wireless
 * bandwidth. It sends one message each time it is called until the buffer is empty.
 * Call this function with xx Hertz to increase/decrease the bandwidth.
 */
void communication_queued_send(void)
{
	//send parameters one by one



	mavlink_msg_param_value_send(MAVLINK_COMM_0,
			(char*) global_data.param_name[m_parameter_i],
			global_data.param[m_parameter_i], MAVLINK_TYPE_FLOAT,
			ONBOARD_PARAM_COUNT, m_parameter_i);   				tel_transmit();
	m_parameter_i++;
}

