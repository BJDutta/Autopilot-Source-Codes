#ifndef CONTROL_H
#define CONTROL_H

#include "lpc_types.h"
#include "config.h"
#include "motor.h"

typedef struct {

	int16_t roll;	// microseconds 0 - 2000
	int16_t pitch; // microseconds 0 - 2000
	int16_t yaw;	// microseconds 0 - 2000

	int16_t throttle; // microseconds 0 - 2000

}rc_t;

extern rc_t rc;

typedef struct {
					// output
	float roll;	// -1 to 1 for hils
	float pitch; // -1 to 1
	float yaw;	// -1 to 1

	float throttle; // 0 to 1

}cs_t;

typedef struct {
	float X;  //
	float Y;
	float Z;
	float vx;	// east speed in m/s
	float vy;	// north speed in m/s
	float vz;   // vertical speed in m/s
	float OH;
	Bool OTT;   // Optitrack Tracking Confidense
}nav_t;


typedef struct {
	uint8_t arm;
	uint8_t alt;
	uint8_t gps;
	uint8_t nav;
}mode_t;

enum ALT_MODE
{
	OFF=0,
	ALT_HOLD = 1,
	VV_CONTROL=2,
	TAKEOFF=3,
	LANDING=4,
};

#define CS_LIMIT  450
#define RP_RATE_LIMIT 300
#define YAW_RATE_LIMIT 200
#define THROT_LIMIT 1900
#define ARMCNTNO  50
#define YAWDEADBAND 30
#define ALTDEADBAND 80


#ifdef HEXA_X			// hexa frame and motor config

//#define MOTOR_NW motors.s2		// pins are GND S4 S3 S2 S1
//#define MOTOR_NE motors.s3
//#define MOTOR_SW motors.s1
//#define MOTOR_SE motors.s4
//
//#define MOTOR_W motors.s5		// pins are  GND Trigger S1 S2 Analog = GND  Trigger S5 S6 Analog
//#define MOTOR_E motors.s6

#define MOTOR_NW motors.s3			// pins are GND S4 S3 S2 S1
#define MOTOR_NE motors.s6
#define MOTOR_SW motors.s5
#define MOTOR_SE motors.s1

#define MOTOR_W motors.s2		// pins are  GND Trigger S1 S2 Analog = GND  Trigger S5 S6 Analog
#define MOTOR_E motors.s4




#else

									// QUAD_X config
#define MOTOR_NW motors.s2		// pins are GND S4 S3 S2 S1
#define MOTOR_NE motors.s3
#define MOTOR_SW motors.s1
#define MOTOR_SE motors.s4

#endif





#define SAILERON  channel[1]
#define SELEVATOR channel[2]
#define STHROTTLE channel[3]
#define SRUDDER   channel[4]
#define ALT_CTRL  channel[5]
#define POT_CTRLR  channel[6]
#define POT_CTRLL  channel[7]

extern cs_t cs;
extern nav_t nav;
extern nav_t targetnav;
extern nav_t home;
extern mode_t RCmode;
extern mode_t CommMode;
extern float homealt;
extern int Low_battery;

extern float sonar_height;

 extern char tablet_armed;

void CS_arm();
void CS_disarm();
void sticks();
void CS_setup();
void modes();
void control_system();
void limitI(float *value, float *lim);
float constrain_float(float value, float lowerlim, float upperlim);
void check_cs_limits();
void check_motor_limits();
void altimeter();




#endif
