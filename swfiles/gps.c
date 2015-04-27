
#include "gps.h"
#include "config.h"
#include "control.h"
#include "imu.h"
#include "lpc_types.h"
#include "lpc17xx_timer.h"
#include "mlink.h"
#include "quat.h"
#include "waypoint.h"
#include "timing.h"
#include "hil.h"
#include "opticflow.h"
#include "quat.h"

char MissionComplete = 0;
int8_t WPtimeout = 0;

char Takeoff=0;
char Landing = 0;

float OptitrackX=0;
float OptitrackY=0;
float OptitrackZ=0;
float OptitrackError=0;
float OptitrackVx =0;
float OptitrackVy=0;
float OptitrackVz=0;
float OptitrackHeading=0;
Bool  OptitrackTracking;


float nroll=0, npitch = 0;


float Iconstrain(float x, float lim){

	if(x>lim)x = lim;
	else if(x<-lim) x = -lim;

	return x;
}

void navigation(){

	static float Xrate=0, Yrate=0;
	static float Inavx = 0, Inavy = 0, Inavn=0;
	float psi=0, cospsi, sinpsi, PIDx, PIDy, PIDn;
	float DTG=0;   // distance to go
	float navangle=0;
	float navrate=0;
	static int WPnumber=0;
	static uint8_t loiter_set = 0;
	static nav_t loiternav;
	float stickroll, stickpitch;
	float K;

	static float dtnav=0;
	static uint32_t lastus = 0, nowus=0;

	nowus = (LPC_TIM2->TC);

	if(nowus<lastus){

		dtnav = ((20000000 + nowus)-lastus)/1000000.0;  // 20s loop timer
	}
	else dtnav =  (nowus-lastus)/1000000.0;

	lastus = nowus;

	//	nav.X = OptitrackX;
	//	nav.Y = OptitrackY;
	//	nav.Z =  OptitrackZ;
	//	nav.vx = OptitrackVx;
	//	nav.vy = OptitrackVy;
	//	nav.vz = OptitrackVz;
	//	nav.OH= OptitrackHeading;
	//	nav.OTT   = OptitrackTracking;

	if(OptitrackTracking == TRUE){

		K = *MISC_MISSION_SIZE;
		INS.Vx= (1-K) * INS.Vx + K * OptitrackVx;
		INS.X = (1-K)*(INS.X) + K* OptitrackX;

		INS.Vy= (1-K) * INS.Vy + K * OptitrackVy;
		INS.Y = (1-K)*(INS.Y) + K * OptitrackY;

		INS.Vz= (1-K) * INS.Vz + K * OptitrackVz;
		INS.Z = (1-K)*(INS.Z) + K * OptitrackZ;
	}

	if(RCmode.gps && OptitrackTracking ){

		psi = OptitrackHeading;
		cospsi = cos(psi*D2R);	sinpsi = sin(psi*D2R);  // feed optitrack yaw here in psi

		DTG = sqrt((targetnav.X-nav.X)*(targetnav.X-nav.X)  + (targetnav.Y-nav.Y)*(targetnav.Y-nav.Y));

		navangle = atan2((targetnav.Y-nav.Y), (targetnav.X-nav.X));

		navrate = Iconstrain((*Kpn)*DTG, MAX_GROUND_SPEED);

		Xrate = navrate*cos(navangle);  // X towards east // nav angle says the direction where the robot as to move
		Yrate = navrate*sin(navangle);	// Y towards north

		if(!loiter_set){
			loiternav.X = nav.X; loiternav.Y = nav.Y;
			loiter_set = 1;
			targetnav.X = loiternav.X; targetnav.Y = loiternav.Y;
		}

		stickroll = (rc.roll - 1500);
		stickpitch = (rc.pitch - 1500);

		if(fabs(stickroll)<NAVDEADBAND) stickroll = 0;		// add diagonal distance of stick as deadband
		if(fabs(stickpitch)<NAVDEADBAND) stickpitch = 0;

		if(stickroll!=0 || stickpitch!=0){

			stickroll*=(*RC_GSENSE)/500.0f;
			stickpitch*=(*RC_GSENSE)/500.0f;

			Xrate = stickpitch*sinpsi + stickroll*cospsi;
			Yrate = stickpitch*cospsi - stickroll*sinpsi;
			loiter_set = 0;
		}

		mavlink_system.nav_mode =(mavlink_system.nav_mode&0xF0)|(1);

		Inavx  = Iconstrain(Inavx, *ILIMIT_XY);
		Inavy  = Iconstrain(Inavy, *ILIMIT_XY);

		Inavn = Iconstrain(Inavn, *ILIMIT_XY);

		Inavx  = Inavx  + (*Kin)*(Xrate-nav.vx)*dtnav;  // feed optitrack speeds
		Inavy  = Inavy  + (*Kin)*(Yrate-nav.vy)*dtnav;




		PIDx = Inavx + (*Kdn)*(Xrate - nav.vx);// + nKd2*Ax;
		PIDy = Inavy + (*Kdn)*(Yrate - nav.vy);// + nKd2*Ay;


		//			PIDn = (navrate - GPS.ground_speed/100.0f);


		nroll = PIDx*cospsi - PIDy*sinpsi; nroll = Iconstrain(nroll, (*ILIMIT_TILT));
		npitch = -PIDx*sinpsi - PIDy*cospsi; npitch = Iconstrain(npitch, (*ILIMIT_TILT));


	}
	else {
		Inavn = Inavx = Inavy = 0;
		nroll = npitch = 0;

		Takeoff = 0;
		Landing = 0;
		loiter_set = 0;

	}
}

void gpscomm_receive(){

	mavlink_message_t msg;
	mavlink_status_t status;


	static unsigned char serBuf [RX_BUF_SIZE];
	int  numBytesRead;
	int index=0;
	static int fixnumber=0;

	numBytesRead = gps_receive(&serBuf[0]);

	if(!numBytesRead) return;

#ifdef  HIL_MODE
	parse_HIL_data(&serBuf[0], numBytesRead);		// enable this for hil

#else
	// COMMUNICATION THROUGH EXTERNAL UART PORT (GPS port)



	while(numBytesRead--)
	{
		uint8_t c = serBuf[index++];
		// Try to get a new message
#ifdef UBLOX_GPS

		if(UBLOX_read(c)== TRUE){
			navigation();
			if(GPS.fix){
				if(fixnumber++>10){
					GPS_blink();
					fixnumber=0;
				}

			}
			else GPS_blink();
		}
#endif

#ifdef PX4_FLOW
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
			// Handle message


			switch(msg.msgid)
			{
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				// E.g. read GCS heartbeat and go into
				// comm lost mode if timer times out
				//										blink();
			}
			break;

			case MAVLINK_MSG_ID_OPTICAL_FLOW:
			{
				GPS_blink();

				//				flow_comp_m_x = mavlink_msg_optical_flow_get_flow_comp_m_x(&msg);	// meters X  is forward
				//				flow_comp_m_y = mavlink_msg_optical_flow_get_flow_comp_m_y(&msg);	// meters Y is rightward
				//
				//				flow_quality = mavlink_msg_optical_flow_get_quality(&msg);			// 0-255 with 255 as best
				//
				//				sonar_height = mavlink_msg_optical_flow_get_ground_distance(&msg);

				//optical_nav();
			}
			break;

			case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			{
				//						GPS_blink();
			}
			case 106:
			{
				//						blink();
			}
			break;
			default:
				//Do nothing
				//								imu.roll= msg.msgid;
				//				blink();
				break;
			}
		}

		// Update global packet drops counter
		//					packet_drops += status.packet_rx_drop_count;

		//					if(status.packet_rx_drop_count)blink();



#endif

	}

#endif

}

