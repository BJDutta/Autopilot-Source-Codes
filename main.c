#include "lpc17xx.h"
#include "config.h"
#include "LED_Blink.h"
#include "timing.h"
#include "comm.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_adc.h"
#include "mlink.h"
#include "mathfunc.h"
#include "imu.h"
#include "control.h"
#include "i2ceeprom.h"
#include "motor.h"
#include "imu.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_wdt.h"
#include "MS5611.h"
#include "gps.h"
#include "ublox.h"
#include "waypoint.h"
#include "chipid.h"
#include "hil.h"
#include "quat.h"

#define WDT_TIMEOUT 2000000
#define WDT_PROGRAM_TIMEOUT 600000000

//#define IAP_LOCATION 0x1FFF1FF1
////
//typedef void (*IAP)(unsigned int [],unsigned int[]);
//IAP iap_entry;

void DisconnectPLL0(){
	// good practice to disable before feeding
	__disable_irq();
	// disconnect
	LPC_SC->PLL0CON = 0x1;
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;
	while (LPC_SC->PLL0STAT&(1<<25));
	// power down
	LPC_SC->PLL0CON = 0x0;
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;
	while (LPC_SC->PLL0STAT&(1<<24));
	// This is the default flash read/write setting for IRC
	LPC_SC->FLASHCFG &= 0x0fff;
	LPC_SC->FLASHCFG |= 0x5000;
	LPC_SC->CCLKCFG = 0x0;
	//  Select the IRC as clk
	LPC_SC->CLKSRCSEL = 0x00;
	// not using XTAL anymore
	LPC_SC->SCS = 0x00;
}

// IAP address
#define IAP_LOCATION 0x1FFF1FF1
// variable for command and result
unsigned int command[5];
unsigned int result[5];
// function pointer with 2 array parameters
typedef void (*IAP)(unsigned int[5],unsigned int [5]);
// declaration of the fptr.
IAP iap_entry = (IAP)IAP_LOCATION;

//uint8_t ISP_Reserved[0x1FF-0x118] __attribute__((at(0x10000118)));

void watchdog_setup(int type){
	/* Install interrupt for WDT interrupt */
	NVIC_SetPriority(WDT_IRQn, 0x10);
	WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
	/* Enable the Watch dog interrupt*/
	NVIC_EnableIRQ(WDT_IRQn);

	/* Start watchdog with timeout given */
	if(type)WDT_Start(WDT_PROGRAM_TIMEOUT);
	else WDT_Start(WDT_TIMEOUT);
}

void BAT_setup(){

	PINSEL_CFG_Type PinCfg;
	//	uint32_t adc_value, tmp;



	/*
	 * Init ADC pin connect
	 * AD0.5 on P1.31
	 */
	PinCfg.Funcnum = 3;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 31;
	PinCfg.Portnum = 1;
	PINSEL_ConfigPin(&PinCfg);


	/* Configuration for ADC :

	 *  ADC conversion rate = 200Khz
	 */
	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN5,DISABLE);
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_5,ENABLE);
}

float BAT_read(){
	static float voltage;
	// Start conversion
	ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	//Wait conversion complete
	while (!(ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_5,ADC_DATA_DONE)));
	voltage = 0.9*voltage + 0.1*(float)(ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_5))*3.3/4096*6.1; // resistor calibration 6.1

	return voltage;
}
int main(void)
{

	//	int triggercounter=0;
	int i=0;
	uint8_t cs_ratio =0;

	//!make IAP call
	command[0] = 58;
	iap_entry(command, result);

	if(!check_id(&result)){
		LED_Setup();
		//		   blink();
		while(1){
			for(i=0;i<3000;i++){
				blink();
				GPS_blink();
				delay(100);
			}

			watchdog_setup(1);
			DisconnectPLL0();
			//! stop all ISRs
			__disable_irq();
			//!make IAP call
			command[0] = 57;
			iap_entry(command, result);

		}
	}


//		watchdog_setup(0);

	i2c_eeprom_init();	WDT_Feed();
	mavlink_init();	WDT_Feed();

//	if((*HW_PROGRAM==1)){
//
//		*HW_PROGRAM = 0;  // reset the program parameter and writing in the eeprom
//		eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));
//		watchdog_setup(1);
//		DisconnectPLL0();
//		//! stop all ISRs
//		__disable_irq();
//		//!make IAP call
//		command[0] = 57;
//		iap_entry(command, result);
//	}



	LED_Setup();	WDT_Feed();

	timing_init();	WDT_Feed();

	comm_init();	WDT_Feed();

//	waypoint_init();WDT_Feed();
#ifdef HIL_MODE

#else

	imu_setup();	WDT_Feed();



//	Baro_init();    WDT_Feed();	// disabled barometer initialization

	PWM_setup();	WDT_Feed();
//	Motors_set();
	BAT_setup();


#endif

	PPM_setup();    WDT_Feed();

	if((LPC_SC->RSID & 0x03)!=0){
		delay(1000);
		//		LPC_SC->RSID = ((LPC_SC->RSID)>>2)<<2;
		LPC_SC->RSID = 0x0F;
		NVIC_SystemReset();
	}


	delay(100);	    WDT_Feed();
	CS_setup();

#ifdef HIL_MODE

#else

	Motors_set();

#ifdef UBLOX_GPS
	_configure_gps();		// to initialize ublox gps
#endif

#endif

	while(1)
	{
		WDT_Feed();
//
//		if(firmware_flash==1){			// flashing the firmware
//			if(RCmode.arm){
//				firmware_flash=0;
//				continue;
//			}
//
//
//			*HW_PROGRAM=1;
//			eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));
//			NVIC_SystemReset();
//		}

		communication_receive();
//		gpscomm_receive();
		timing();

		if(IMUhzloopflag){
			IMUhzloopflag=0;
#ifdef HIL_MODE

#else
			imu_update();
#endif
			modes();
			control_system();
			if(cs_ratio++ >= 1){
				cs_ratio=0;
				altimeter();
			}



#ifdef HIL_MODE
			send_HIL();
#else

			Motors_set();
#endif

			if(RCmode.arm)blink(); // to indicate arm mode with almost solid LED
		}

		if(onehzloopflag){
			onehzloopflag=0;


#ifdef HIL_MODE
			    		blink();
#endif
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, mavlink_system.type, MAV_AUTOPILOT_GENERIC, mavlink_system.mode, mavlink_system.nav_mode, mavlink_system.state); //tel_transmit();
			mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, mavlink_status.load , mavlink_status.voltage_battery, -1, mavlink_status.battery_remaining, mavlink_status.drop_rate_comm, mavlink_status.errors_comm, mavlink_status.errors_count1, mavlink_status.errors_count2, mavlink_status.errors_count3, mavlink_status.errors_count4); //tel_transmit();
//			tel_transmit();
			if(!fivehzloopflag) tel_transmit();

		}

//		if(hundredhzloopflag){
//			hundredhzloopflag=0;
//
//		}

		if(fiftyhzloopflag){
			fiftyhzloopflag=0;

			sticks();
//			boscam_pwm();

		}

		if(tenhzloopflag){
			tenhzloopflag=0;

#ifdef HIL_MODE

#else
			mavlink_status.voltage_battery = BAT_read()*1000;
			mavlink_status.battery_remaining = (int8_t)(100 - (4120-(float)mavlink_status.voltage_battery/3.0)/5.14);
			if(mavlink_status.battery_remaining>100)mavlink_status.battery_remaining=100;

#endif
//
//			if(mavlink_status.battery_remaining<5){
////				Low_battery++;
//			}

		}

		if(twentyhzloopflag){
			twentyhzloopflag=0;
			//    		mavlink_msg_hil_controls_send(MAVLINK_COMM_0, micros(), cs.roll, cs.pitch, cs.yaw, cs.throttle, 0, 0, 0, 0, mavlink_system.mode, mode); tel_transmit();
			if (m_parameter_i < ONBOARD_PARAM_COUNT) communication_queued_send();
		}

		if(fivehzloopflag){
			fivehzloopflag=0;

			if(*FIELD_GEN>=1){
				nav_generate(*FIELD_GEN);
			}




			mavlink_msg_attitude_send(MAVLINK_COMM_0, millisnow, d2r*imu.roll, d2r*imu.pitch, d2r*imu.yaw, d2r*imu.rollrate, d2r*imu.pitchrate, d2r*imu.yawrate); //tel_transmit();

           //mavlink_msg_global_position_int_send(MAVLINK_COMM_0, millisnow, nav.lat, nav.lon, (nav.alt + homealt)*1000, nav.alt*1000, GPS._vel_north, GPS._vel_east, nav.vz*100, GPS.ground_course); //tel_transmit();
			mavlink_msg_global_position_int_send(MAVLINK_COMM_0, millisnow, nav.X*1000, nav.Y*1000, nav.Z*1000, OptitrackError*100000, nav.vx*100, nav.vy*100,nav.vz*100,OptitrackHeading*100);
			mavlink_msg_setpoint_6dof_send(MAVLINK_COMM_0, mavlink_system.sysid, GPS.hdop/1000.0f, GPS.num_sats, targetnav.Z, D2R*AX_E, D2R*AY_E, D2R*AZ_E);//D2R*cstarget.roll, D2R*cstarget.pitch, D2R*cstarget.yaw); //tel_transmit();
			mavlink_msg_set_quad_motors_setpoint_send(MAVLINK_COMM_0, mavlink_system.sysid, cs.roll, cs.pitch, cs.yaw, cs.throttle);//MOTOR_SW);
			mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0,millisnow, 0, channel[0],channel[1],channel[2],channel[3],channel[4],channel[5],channel[6],channel[7], 50);	tel_transmit();
		}
	}
}

