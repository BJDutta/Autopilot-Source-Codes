#ifndef GPS_H
#define GPS_H

#include "ublox.h"
#include "comm.h"
#include "mlink.h"
#include "LED_Blink.h"


#define Re 6378500   // radius of earth in meters
#define ToRad 0.01745329
#define ToDeg 57.2957795

#define NAVDEADBAND 40
#define MAX_GROUND_SPEED .6

enum NAV_MODE
{
	FREE=0,
	LOITER=1,
	VELOCITY_CONTROL=2,
	RETURN_TO_HOME=3,
};


extern char MissionComplete;
extern int8_t WPtimeout;
extern char Takeoff, Landing;
extern float nroll, npitch;
extern float OptitrackX,OptitrackY,OptitrackZ,OptitrackError,OptitrackVx,OptitrackVy,OptitrackVz,OptitrackHeading;
extern Bool OptitrackTracking;

float Iconstrain(float x, float lim);
void navigation();
void gpscomm_receive();



#endif
