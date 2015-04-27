#include "opticflow.h"
#include "timing.h"
#include "gps.h"
#include "control.h"


float flow_comp_m_x =0, flow_comp_m_y=0;
uint8_t flow_quality = 0;


void optical_nav(){

	static float X=0, Y=0, Xrate=0, Yrate=0;   // Xrate is in body frame with X being forward and Y being rightward
	static float Inavx = 0, Inavy = 0, Inavn=0;
	float psi=0, cospsi, sinpsi, PIDx, PIDy, PIDn;
	float DTG=0;   // distance to go
	float navangle=0;
	float navrate=0;

	static uint8_t loiter_set = 0;
	static nav_t loiternav;
	float stickroll, stickpitch;

	static float dtoptic=0;
	static uint32_t lastus = 0, nowus=0;

	nowus = (LPC_TIM2->TC);

	if(nowus<lastus){

		dtoptic = ((20000000 + nowus)-lastus)/1000000.0;  // 20s loop timer
	}
	else dtoptic =  (nowus-lastus)/1000000.0;

	lastus = nowus;


	switch(RCmode.nav){
	case LOITER:
	{
		if(!loiter_set){
			loiternav.lat = nav.lat; loiternav.lon = nav.lon;
			loiter_set = 1;
			targetnav.lat = loiternav.lat; targetnav.lon = loiternav.lon;
		}

		stickroll = (rc.roll - 1500);
		stickpitch = (rc.pitch - 1500);

		if(fabs(stickroll)<NAVDEADBAND) stickroll = 0;		// add diagonal distance of stick as deadband
		if(fabs(stickpitch)<NAVDEADBAND) stickpitch = 0;

		stickroll*=(*RC_GSENSE)/500.0f;
		stickpitch*=(*RC_GSENSE)/500.0f;

		if(stickroll!=0 || stickpitch!=0){
			Xrate = stickpitch;
			Yrate = stickroll;
			loiter_set = 0;
		}

		mavlink_system.nav_mode =(mavlink_system.nav_mode&0xF0)|(1);
	}
	break;
	//			case VELOCITY_CONTROL:
	//			{
	//				loiter_set = 0;
	//				mavlink_system.nav_mode =(mavlink_system.nav_mode&0xF0)|(1<<1);
	//			}
	//			break;
	//			case RETURN_TO_HOME:
	//			{
	//				targetnav.lat = home.lat; targetnav.lon = home.lon;
	//				loiter_set = 0;
	//				mavlink_system.nav_mode =(mavlink_system.nav_mode&0xF0)|(1<<2);
	//			}
	//			break;
	default:
		loiter_set = 0;
		break;

	}


	//			Inavx  = Inavx  + (*Kin)*(Xrate-GPS._vel_east/100.00)*dtnav;
	//			Inavy  = Inavy  + (*Kin)*(Yrate-GPS._vel_north/100.00)*dtnav;
	//
	//			Inavx  = Iconstrain(Inavx, *ILIMIT_XY);
	//			Inavy  = Iconstrain(Inavy, *ILIMIT_XY);
	//
	//			Inavn = Iconstrain(Inavn, *ILIMIT_XY);
	//
	//
	//			PIDx = Inavx + (*Kdn)*(Xrate - GPS._vel_east/100.0f);// + nKd2*Ax;
	//			PIDy = Inavy + (*Kdn)*(Yrate - GPS._vel_north/100.0f);// + nKd2*Ay;
	//
	//
	//			PIDn = (navrate - GPS.ground_speed/100.0f);

	//
	//			nroll = PIDx*cospsi - PIDy*sinpsi; nroll = Iconstrain(nroll, (*ILIMIT_TILT));
	//			npitch = -PIDx*sinpsi - PIDy*cospsi; npitch = Iconstrain(npitch, (*ILIMIT_TILT));


	//			nroll = *Kdn*(Yrate - flow_comp_m_y); 		nroll = Iconstrain(nroll, (*ILIMIT_TILT));
	//			npitch = *Kdn*(Xrate - flow_comp_m_x); 					npitch = Iconstrain(npitch, (*ILIMIT_TILT));

	nroll = ( flow_comp_m_y); 		nroll = Iconstrain(nroll, (*ILIMIT_TILT));
	npitch = ( flow_comp_m_x); 					npitch = Iconstrain(npitch, (*ILIMIT_TILT));
}


