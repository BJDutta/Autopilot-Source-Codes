

#include "control.h"
#include "config.h"
#include "mlink.h"
#include "param.h"
#include "imu.h"
#include "timing.h"
#include "math.h"
#include "stdlib.h"
#include "MS5611.h"
#include "quat.h"
#include "gps.h"
#include "opticflow.h"

rc_t rc; // RC transmitter sticks control
cs_t cs; // Target Control System

nav_t nav;
nav_t targetnav;
nav_t home;

mode_t RCmode;
mode_t CommMode;

char tablet_armed = 0;
char hil_mode = 0;  // hil mode

float homealt = 0;
float Ialt = 0 ;
int AltThrottle = 500;
int HoverThrottle = 1000;		// hover pwm value for motors // microseconds Value
int HoverStickValue = 1500;  // throttle stick value for altitude target control through RC
float lastalt=0;
float vv_target=0;	// for vertical speed control mode

float sonar_height = 0; // from the px4flow sensor  , -ve is unknown distance

uint8_t Shutdown = 0;  // to disarm motors after touchdown
int32_t armcount=0;

int Low_battery=0;

void CS_arm(){		// arm through tablet

	if(!GPS.fix) return;   // POP an error of GPS not fixed, if armed from tablet

	homealt = nav.Z + homealt;
#ifdef HIL_MODE

#else
	gyro_calibrate();
#endif

	RCmode.arm = 1;
	armcount = ARMCNTNO;

	tablet_armed = 1;

}

void CS_disarm(){

	RCmode.arm = 0;
	armcount = 0;

	rc.throttle = *MOTOR_MIN - 50;

	tablet_armed = 0;
}

void sticks(){

	static int index=0;
	static float gain = 0;
	static int tempthrottle = 0;

	rc.roll  = SAILERON;  // RC channels defined
	rc.pitch = SELEVATOR;
	rc.yaw   = SRUDDER;
	rc.throttle = STHROTTLE;


	//	if(rc.throttle<800)return;		// this line will disable all further calculations if there is no RC receiver connected



	if(Shutdown == 1 && RCmode.arm){  // disarm motors on shutdown... relocate this code to control_system on removal of sticks
		RCmode.arm = 0;
		tablet_armed = 0;
		armcount = 0;
		Shutdown = 0;
		RCmode.alt = 0;
		RCmode.gps = 0;
		RCmode.nav = 0;
		rc.throttle = *MOTOR_MIN-50;
	}

	//	if(POT_CTRLR>1500 || STHROTTLE<800){		// comment these lines for removing tablet control ability for channel 6
	//		rc.throttle = *MOTOR_MIN - 5;
	//		return;
	//	}

	tablet_armed = 0;

	if(rc.throttle< 1070 && rc.throttle>900){
		Shutdown = 0;
		if(rc.yaw > 1800 && rc.yaw < 2050 && !RCmode.arm) armcount++;
		else if(rc.yaw < 1200 && rc.yaw>950 && RCmode.arm) armcount--;

		if(armcount>=ARMCNTNO && !RCmode.arm){		// manual arming through RC
			RCmode.arm = 1;
			tablet_armed = 0;
			homealt = nav.Z + homealt;
			armcount = ARMCNTNO;
#ifdef HIL_MODE
#else
			gyro_calibrate();
#endif
		}
		else{
			if(armcount<=0 && RCmode.arm){
				RCmode.arm = 0;
				tablet_armed = 0;
				armcount = 0;
			}
		}
	}


	index = *RC_PARAM_IDXL;

	if(index>0 && index<12) {  // 12 number of PIDs
		gain = *RC_PARAM_MINL + (POT_CTRLL-1000)*(*RC_PARAM_MAXL - *RC_PARAM_MINL)/1000.0f;
		if(gain>*RC_PARAM_MAXL) gain = *RC_PARAM_MAXL;
		else if(gain<*RC_PARAM_MINL) gain = *RC_PARAM_MINL;
		global_data.param[index-1] = gain;
	}


	//	if(POT_CTRLR>1500 && !RCmode.nav)trigger(5000);


	//	if(POT_CTRLR>1500){
	//		Takeoff = 1;
	//		Landing = 1;
	//
	//
	//	}


	index = (int)*RC_PARAM_IDXR;

	if(index>0 && index<=12) {  // 12 number of PIDs
		gain = *RC_PARAM_MINR + (POT_CTRLR-1000)*(*RC_PARAM_MAXR - *RC_PARAM_MINR)/1000.0f;
		if(gain>*RC_PARAM_MAXR) gain = *RC_PARAM_MAXR;
		else if(gain<*RC_PARAM_MINR) gain = *RC_PARAM_MINR;
		global_data.param[index-1] = gain;
	}


	if(abs(rc.yaw-1500)>YAWDEADBAND && rc.throttle>*MOTOR_MIN){
		cstarget.yaw = cstarget.yaw + (*RC_YSENSE)*(rc.yaw - 1500)/1000.0f;
		if(cstarget.yaw>180)cstarget.yaw -=360;
		if(cstarget.yaw<=-180)cstarget.yaw+=360;
	}


	if(RCmode.alt!=OFF){
		tempthrottle = rc.throttle - HoverStickValue;
		switch(RCmode.alt){
		case ALT_HOLD: {
			if(abs(tempthrottle)>ALTDEADBAND)targetnav.Z += (*RC_ASENSE)*(tempthrottle)/20000.0f; // 12.5 cm/s to 1.25 m/s for 50 to 500 us
		}
		break;
		case VV_CONTROL:{
			if(abs(tempthrottle)>ALTDEADBAND){
				vv_target= (*RC_ASENSE)*(tempthrottle)/200.0f; //
			}
			else{
				vv_target = 0;
			}
		}
		default: break;
		}
	}
	else {
		HoverStickValue = rc.throttle;
	}
}

void CS_setup(){

	RCmode.alt = OFF;
	RCmode.arm = 0;
	RCmode.gps = 0;
	RCmode.nav = FREE;

	CommMode.alt = VV_CONTROL;
	//	CommMode.arm = 0;
	CommMode.gps = 1;
	CommMode.nav = LOITER;
}

void modes(){


	if(ALT_CTRL>1250 && ALT_CTRL<1750){		// Position hold mode  position 2 on channel 5

		RCmode.alt = VV_CONTROL ;
		RCmode.gps = 0;

		cstarget.roll =    (rc.roll - 1500)/(*RC_SENSE);
		cstarget.pitch =   -(rc.pitch - 1500)/(*RC_SENSE);

		cstarget.roll = constrain(cstarget.roll, -45,45);
		cstarget.pitch = constrain(cstarget.pitch, -45,45);

		CommMode.alt = VV_CONTROL;   // reset the CommMode to default
		CommMode.gps = 1;
		CommMode.nav = LOITER;

		mavlink_system.mode = ((RCmode.arm<<7)|(1<<3));  // armed, altitude hold

	}
	else{
		if(ALT_CTRL>=1750 && ALT_CTRL<2200){		// NAV mode  position 3 on channel 5

			RCmode.alt = VV_CONTROL ;
			RCmode.gps = LOITER;
			cstarget.roll =    nroll;
			cstarget.pitch =   npitch;

			cstarget.roll = constrain(cstarget.roll, -35,35);
			cstarget.pitch = constrain(cstarget.pitch, -35,35);

			CommMode.alt = VV_CONTROL;			// reset the CommMode to default
			CommMode.gps = 1;
			CommMode.nav = LOITER;

			mavlink_system.mode = ((RCmode.arm<<7)|(1<<2));  // armed, commanded
		}
		else {
			RCmode.alt = OFF;		// direct manual mode, stabilized   position 1 on channel 5
			RCmode.gps = 0;
			RCmode.nav = 0;

			cstarget.roll =    (rc.roll - 1500)/(*RC_SENSE);
			cstarget.pitch =   -(rc.pitch - 1500)/(*RC_SENSE);

			cstarget.roll = constrain(cstarget.roll, -45,45);
			cstarget.pitch = constrain(cstarget.pitch, -45,45);

			CommMode.alt = VV_CONTROL;   // reset the CommMode to default
			CommMode.gps = 1;
			CommMode.nav = LOITER;

			mavlink_system.mode = ((RCmode.arm<<7)|(1<<4));  // armed, stabilized
		}
	}


	//	mavlink_system.nav_mode = ((RCmode.nav&0x03)<<6)|((RCmode.gps&0x03)<<4)|((RCmode.alt&0x03)<<2)|(RCmode.arm&0x03);// two bits for each mode type

}


void control_system(){

	//	float tempthrottle;
	//	static float deadband=1;
	//	static uint32_t millislast = 0;
	static float Iroll=0, Ipitch=0, Iyaw=0;
	//	static float dalt = 0;
	//	static float altitudelast=0;
	//	float dt=0;
	//	float yawerr=0;

	static float rollerror=0, pitcherror=0, yawerror = 0, rollrate_E=0, pitchrate_E=0, yawrate_E=0,rollrate_B=0, pitchrate_B=0, yawrate_B=0;
	static float dt =0.01;
	static float sin_pitch, sin_roll, cos_pitch, cos_roll;
	static uint32_t lastus = 0, nowus=0;


	nowus = (LPC_TIM2->TC); //us- microseconds

	if(nowus<lastus){

		dt = ((20000000 + nowus)-lastus)/1000000.0;  // 20s loop timer
	}
	else dt =  (nowus-lastus)/1000000.0;

	lastus = nowus;



	if(rc.throttle < (*MOTOR_MIN) && !RCmode.alt){
		if(tablet_armed==1){
			motors.s1 = motors.s2 = motors.s3 = motors.s4 = (*MOTOR_MIN) - 15.0f;		// run motors at slow speed on tablet armed
		}
		else{
			motors.s1 = motors.s2 = motors.s3 = motors.s4 = rc.throttle;		// follow rc throttle on manual arm
		}


		// integrals zero

		cs.throttle = rc.throttle;

		cstarget.yaw = imu.yaw;

		Iroll = Ipitch = Iyaw = Ialt = 0;
		HoverThrottle = rc.throttle;
		targetnav.Z = nav.Z;


	}
	else{

		yawerror = (cstarget.yaw - imu.yaw);

		if(yawerror>180)yawerror-=360;
		if(yawerror<=-180)yawerror +=360;			// yaw is on new pid control system

#ifdef HIL_MODE

		rollerror  = (cstarget.roll-imu.roll);
		pitcherror = (cstarget.pitch-imu.pitch);


		rollrate_E = rollerror;
		pitchrate_E = pitcherror;
		yawrate_E = yawerror;

		yawrate_E = constrain_float(yawrate_E, -40,40);

		// remember to add body angle errors to body frame rate targets

		sin_pitch = sin(D2R*imu.pitch); sin_roll = sin(D2R*imu.roll);
		cos_pitch = cos(D2R*imu.pitch); cos_roll = cos(D2R*imu.roll);

		// convert earth frame rates to body frame rates
		rollrate_B  = rollrate_E - sin_pitch * yawrate_E;
		pitchrate_B = cos_roll  * pitchrate_E + sin_roll * cos_pitch * yawrate_E;
		yawrate_B   = -sin_roll * pitchrate_E + cos_pitch * cos_roll * yawrate_E;


		rollerror = rollrate_B;
		pitcherror = pitchrate_B;
		yawerror   = yawrate_B;



		Iroll  = Iroll  + (*Ki)*rollerror*dt;  limitI(&Iroll, ILIMIT_RP); // parameter use of *ILIMIT_RP
		Ipitch = Ipitch + (*Ki)*pitcherror*dt; limitI(&Ipitch, ILIMIT_RP);
		Iyaw   = Iyaw   + (*Kiy)*yawerror*dt;  limitI(&Iyaw, ILIMIT_Y);



		cs.roll  = (*Kp)*rollerror  + Iroll  - ((*Kd)*imu.rollrate);
		cs.pitch = (*Kp)*pitcherror + Ipitch - ((*Kd)*imu.pitchrate);

		cs.yaw = (*Kpy)*(yawerror) + Iyaw - (*Kdy)*(imu.yawrate);

#else

		rollrate_E  = (*Kp)*(cstarget.roll-imu.roll);
		pitchrate_E = (*Kp)*(cstarget.pitch-imu.pitch);
		yawrate_E   = (*Kpy)*(yawerror);

		rollrate_E = constrain_float(rollrate_E, -RP_RATE_LIMIT, RP_RATE_LIMIT);
		pitchrate_E = constrain_float(pitchrate_E, -RP_RATE_LIMIT, RP_RATE_LIMIT);
		yawrate_E = constrain_float(yawrate_E, -YAW_RATE_LIMIT,YAW_RATE_LIMIT);

		// remember to add body angle errors to body frame rate targets

		sin_pitch = sin(D2R*imu.pitch); sin_roll = sin(D2R*imu.roll);
		cos_pitch = cos(D2R*imu.pitch); cos_roll = cos(D2R*imu.roll);

		// convert earth frame rates to body frame rates
		rollrate_B  = rollrate_E - sin_pitch * yawrate_E;
		pitchrate_B = cos_roll  * pitchrate_E + sin_roll * cos_pitch * yawrate_E;
		yawrate_B   = -sin_roll * pitchrate_E + cos_pitch * cos_roll * yawrate_E;


		rollerror = rollrate_B - imu.rollrate;
		pitcherror = pitchrate_B - imu.pitchrate;
		yawerror   = yawrate_B - imu.yawrate;




		Iroll  = Iroll  + (*Ki)*rollerror*dt;  limitI(&Iroll, ILIMIT_RP); // parameter use of *ILIMIT_RP
		Ipitch = Ipitch + (*Ki)*pitcherror*dt; limitI(&Ipitch, ILIMIT_RP);
		Iyaw   = Iyaw   + (*Kiy)*yawerror*dt;  limitI(&Iyaw, ILIMIT_Y);



		cs.roll  = (*Kd)*rollerror  + Iroll;
		cs.pitch = (*Kd)*pitcherror + Ipitch;


		cs.yaw = (*Kdy)*(yawerror) + Iyaw;


#endif

		if(RCmode.alt!=FREE){ // altitude hold on
			cs.throttle = AltThrottle;
		}
		else {
			cs.throttle = rc.throttle;
			Ialt = 0;
			HoverThrottle = rc.throttle;
			targetnav.Z = nav.Z;
		}

		//		//compensation to keep force in z-direction
		//		static float zcompensation;
		//		if (fabs(D2R*imu.roll) > 0.5)
		//		{
		//			zcompensation = 1.13949393;
		//		}
		//		else
		//		{
		//			zcompensation = 1 / cos(D2R*imu.roll);
		//		}
		//		if (fabs(D2R*imu.pitch) > 0.5)
		//		{
		//			zcompensation *= 1.13949393;
		//		}
		//		else
		//		{
		//			zcompensation *= 1 / cos(D2R*imu.pitch);
		//		}
		//
		//		zcompensation = constrain_float(zcompensation, 1,1.30);		// limit compensation to 30% of hovering throttle
		//
		//#ifdef HIL_MODE
		//
		//#else
		//		zcompensation = sqrtf(zcompensation);
		//
		//#endif
		//		cs.throttle = *MOTOR_MIN + (cs.throttle - *MOTOR_MIN)*zcompensation;

		check_cs_limits();

#ifdef HEXA_X

		// HEXA_X (symmetric) config motor control

		MOTOR_NW = cs.throttle + cs.pitch + 0.5*cs.roll - cs.yaw;  // CW
		MOTOR_NE = cs.throttle + cs.pitch - 0.5*cs.roll + cs.yaw;  // CCW
		MOTOR_SW = cs.throttle - cs.pitch + 0.5*cs.roll - cs.yaw;  // CW
		MOTOR_SE = cs.throttle - cs.pitch - 0.5*cs.roll + cs.yaw; // CCW


		MOTOR_W = cs.throttle  + cs.roll + cs.yaw;				// CCW
		MOTOR_E = cs.throttle  - cs.roll - cs.yaw;				// CW

#else

		// QUAD_X config motor control

		MOTOR_NW = cs.throttle + cs.pitch + cs.roll - cs.yaw;
		MOTOR_NE = cs.throttle + cs.pitch - cs.roll + cs.yaw;
		MOTOR_SW = cs.throttle - cs.pitch + cs.roll + cs.yaw;
		MOTOR_SE = cs.throttle - cs.pitch - cs.roll - cs.yaw;

#endif

	}


	check_motor_limits();

}

void limitI(float *value, float *lim){

	if(*value>*lim) *value = *lim;
	else if(*value<-*lim) *value = -*lim;
}
float constrain_float(float value, float lowerlim, float upperlim){
	if(value < lowerlim)value = lowerlim;
	if(value > upperlim)value = upperlim;

	return value;
}
void check_cs_limits(){

	if(cs.roll>CS_LIMIT)cs.roll = CS_LIMIT;
	else if(cs.roll<-CS_LIMIT)cs.roll = -CS_LIMIT;

	if(cs.pitch>CS_LIMIT)cs.pitch = CS_LIMIT;
	else if(cs.pitch<-CS_LIMIT)cs.pitch = -CS_LIMIT;

	if(cs.yaw>CS_LIMIT)cs.yaw = CS_LIMIT;
	else if(cs.yaw<-CS_LIMIT)cs.yaw = -CS_LIMIT;

	if(cs.throttle>THROT_LIMIT)cs.throttle = THROT_LIMIT;
	else if(cs.throttle <*MOTOR_MIN) cs.throttle = *MOTOR_MIN;
}

void check_motor_limits(){
	if(rc.throttle < (*MOTOR_MIN) && !RCmode.alt){
		if(tablet_armed==1){
			motors.s1 = motors.s2 = motors.s3 = motors.s4 = (*MOTOR_MIN) - 15.0f;		// run motors at slow speed on tablet armed
#ifdef HEXA_X
			motors.s5 = motors.s6 = (*MOTOR_MIN) - 15.0f;		// run motors at slow speed on tablet armed
#endif
			return;
		}
		else{
			motors.s1 = motors.s2 = motors.s3 = motors.s4 = rc.throttle;		// follow rc throttle on manual arm

#ifdef HEXA_X
			motors.s5 = motors.s6 = rc.throttle;		// follow rc throttle on manual arm
#endif
			return;
		}
	}

	if(motors.s1<(*MOTOR_MIN))motors.s1 = (*MOTOR_MIN);
	else if(motors.s1>(*MOTOR_MAX))motors.s1 = (*MOTOR_MAX);

	if(motors.s2<(*MOTOR_MIN))motors.s2 = (*MOTOR_MIN);
	else if(motors.s2>(*MOTOR_MAX))motors.s2 = (*MOTOR_MAX);

	if(motors.s3<(*MOTOR_MIN))motors.s3 = (*MOTOR_MIN);
	else if(motors.s3>(*MOTOR_MAX))motors.s3 = (*MOTOR_MAX);

	if(motors.s4<(*MOTOR_MIN))motors.s4 = (*MOTOR_MIN);
	else if(motors.s4>(*MOTOR_MAX))motors.s4 = (*MOTOR_MAX);

#ifdef HEXA_X
	if(motors.s5<(*MOTOR_MIN))motors.s5 = (*MOTOR_MIN);
	else if(motors.s5>(*MOTOR_MAX))motors.s5 = (*MOTOR_MAX);

	if(motors.s6<(*MOTOR_MIN))motors.s6 = (*MOTOR_MIN);
	else if(motors.s6>(*MOTOR_MAX))motors.s6 = (*MOTOR_MAX);
#endif


}

void altimeter(){
	static float dt =0.01, temp=0;
	static uint32_t lastus = 0, nowus=0;
	//	static float lastalt = 0;
	static float alterror = 0;

	static uint8_t LandIaltset=0;
	static float LandIalt = 0;
	float landifactor = 1;
	float climbrate = 0;
	float vrate=0;

	nowus = (LPC_TIM2->TC);

	if(nowus<lastus){

		dt = ((20000000 + nowus)-lastus)/1000000.0;  // 20s loop timer
	}
	else dt =  (nowus-lastus)/1000000.0;

	lastus = nowus;

#ifdef HIL_MODE

#else

#ifdef MS5611_BARO
	temp = Baro_update(dt*1000);																// HIL disable

	nav.alt = nav.alt*0.8 + 0.2*(temp - homealt);												// HIL disable

	nav.Z = accvel = accvel*0.97 + 0.03*vario(); //(nav.alt - lastalt)/dt;  lastalt = nav.alt; // HIL disable]

#else
//#ifdef PX4FLOW_SONAR
//	nav.alt = sonar_height;
//	nav.Z = accvel = accvel*0.9 + 0.1*(nav.alt - lastalt)/dt;  lastalt = nav.alt;
//
//#endif

//	nav.Z = OptitrackZ;
//	nav.vz = accvel = accvel*0.9 + 0.1*OptitrackVz;  lastalt = nav.Z;

#endif

#endif

	alterror = targetnav.Z - nav.Z;

	climbrate = alterror*(*Kpa)/100.0; // gain in cm/s


	switch (RCmode.alt){

	case OFF:
		break;
	case ALT_HOLD:
	{
		mavlink_system.nav_mode =(mavlink_system.nav_mode&0x0F)|(1<<4);
	}
	break;
	case TAKEOFF:
	{
		if(nav.Z<1.5)climbrate = 3; // at take off climbrate is 3 m/s
		HoverThrottle = 1270;

		targetnav.Z = *MISC_NAVALT+1.0;

		if(nav.Z>(targetnav.Z-0.25)){
			RCmode.alt=ALT_HOLD;				// Take-off cleared
			// send a take-off cleared message
		}
		mavlink_system.nav_mode =(mavlink_system.nav_mode&0x0F)|(1<<5);
	}
	break;
	case LANDING:
	{

		if(nav.Z<3){
			climbrate = -0.250;

			if(nav.Z<0.075) { // touch down detection

				RCmode.alt=ALT_HOLD;

				targetnav.Z = nav.Z;
				Shutdown = 1;

			}
		}
		else{
			targetnav.Z = 0;
		}
		mavlink_system.nav_mode =(mavlink_system.nav_mode&0x0F)|(1<<6);
	}
	break;
	case VV_CONTROL:
	{
		if(vv_target!=0){
			targetnav.Z = nav.Z;		// set the target alt to current alt
			climbrate = vv_target;
		}


		mavlink_system.nav_mode =(mavlink_system.nav_mode&0x0F)|(1<<7);
	}
	break;
	default:
		break;
	}


	if(climbrate>0.8)climbrate = 0.8;		// climbrate limit
	else if(climbrate<-0.8)climbrate = -0.8;



	Ialt = Ialt + (*Kia)*(climbrate - nav.vz)*dt;  limitI(&Ialt, ILIMIT_ALT);

	AltThrottle = HoverThrottle + (*Kda)*(climbrate - nav.vz) + Ialt;

	//		AltThrottle = HoverThrottle + (*Kda)*(climbrate - nav.Z) + Ialt;	// enable this line for altitude control testing (manual hover throttle)

}
