#include "hil.h"
#include "lpc_types.h"
#include "comm.h"
#include "lpc17xx_uart.h"
#include "gps.h"
#include "imu.h"
#include "control.h"
#include "LED_Blink.h"

#define HIL_DATA_SIZE 6

unsigned char hil_buff[50];

unsigned char hil_rx_cka = 0,	hil_rx_ckb = 0;	// checksum for hil packet received

unsigned char hil_packet_set=0, HIL=0, HIL1=0, hil_buff_index=0, hil_packet_size=0;


void send_HIL()
{

	char a;
	char b;
	static uint8_t buffer_in[25];
	int throttle = cs.throttle, elevator = (1500- cs.pitch), rudder = (1500+ cs.yaw), aileron = (1500+ cs.roll);
	int count_in = 0;
	buffer_in[count_in++] = 255;

	if(RCmode.arm==0){
		throttle = 1000;
		elevator = rudder = aileron = 1500;
	}

	a = throttle/256;
	b = throttle%256;
	buffer_in[count_in++] = a;
	buffer_in[count_in++] = b;

	a = elevator/256;
	b = elevator%256;
	buffer_in[count_in++] = a;
	buffer_in[count_in++] = b;

	a = rudder/256;
	b = rudder%256;
	buffer_in[count_in++] = a;
	buffer_in[count_in++] = b;

	a = aileron/256;
	b = aileron%256;
	buffer_in[count_in++] = a;
	buffer_in[count_in++] = b;

	UART_Send((LPC_UART_TypeDef *)GPS_UART, buffer_in , count_in, BLOCKING);

}

void parse_HIL_data(unsigned char buff[], int len)
{

	char data;
	int index=0;
	while(len--)
	{
		data = buff[index++];

		if((data == 254 || data == 255) && hil_packet_set==0)
		{
			hil_packet_set = 1;
			hil_buff_index = 0;
			if(data==254)
			{
				HIL1=1;
				hil_rx_cka = 254;
				hil_rx_ckb = 254;
			}
			else
			{
				HIL1=0;
				hil_rx_cka = 255;
				hil_rx_ckb = 255;
			}
		}
		else
		{
			if(hil_packet_set==1)
			{
				if(hil_buff_index==0)
				{
					hil_packet_size = data;
				}
				hil_buff[hil_buff_index] = data;
				hil_rx_cka += data;
				hil_rx_ckb += hil_rx_cka;

				if(++hil_buff_index == hil_packet_size)
				{

					hil_packet_set=0;
					hil_buff_index=0;
					hil_rx_ckb-=hil_rx_cka;
					hil_rx_cka-=data;
					hil_rx_ckb-=hil_rx_cka;
					hil_rx_cka-=hil_buff[hil_packet_size-2];
					if(data==hil_rx_ckb && hil_buff[hil_packet_size-2] == hil_rx_cka)
					{

						if(HIL1==1)
						{
							HIL = 1;
							HIL1 = 0;
						}
						else HIL = 0;

						calculate_hil();
					}
					// else blink();
				}
			}
		}
	}
}
void calculate_hil(){
	if(hil_buff[1]==1)calculate_hil_imu();
	if(hil_buff[1]==2)calculate_hil_gps();
}

void calculate_hil_gps()
{
	uint8_t i;
	static long latlng=0;
	static float ground_speed=0;
	GPS.fix  = hil_buff[16];
	//blink();
	if(GPS.fix==1)
	{

		latlng = 0;

		for(i=2; i<6; i++)
			latlng = latlng*256 + hil_buff[i];

		GPS.longitude=latlng;
		latlng = 0;

		for(i=6; i<10; i++)
			latlng = latlng*256 + hil_buff[i];
		GPS.latitude=latlng;

		GPS.altitude = nav.Z = ((int16_t)((uint16_t)hil_buff[10]<<8) + hil_buff[11])/10.0f;
//		ground_speed = ((int)hil_buff[12]*256 + hil_buff[13]);    // cm/s		// resolve the components to vel_north and east
//		GPS.ground_course = ((int)hil_buff[14]*256 + hil_buff[15])/100.0; // degrees
//
//		if(GPS.ground_course>180)
//			GPS.ground_course-=360;
//
//		GPS._vel_east = ground_speed*sin((GPS.ground_course)*ToRad); //cm/s
//		GPS._vel_north = ground_speed*cos((GPS.ground_course)*ToRad); //cm/s


		GPS._vel_east = (int16_t)(((uint16_t)hil_buff[12]<<8) + hil_buff[13]);    // cm/s

		GPS._vel_north = (int16_t)(((uint16_t)hil_buff[14]<<8) + hil_buff[15]);    // cm/s

		GPS.ground_speed = sqrtf(GPS._vel_east*GPS._vel_east + GPS._vel_north*GPS._vel_north);	// 2D speed magnitude // cm/s

		GPS.ground_course = 100*R2D*atan2f((float)GPS._vel_north, (float)GPS._vel_east);


		GPS.num_sats = hil_buff[17];
		GPS.hdop = 2500;



		nav.vz = 0.5*nav.vz + 0.5*((int16_t)((uint16_t)hil_buff[18]<<8) + hil_buff[19])*0.01;    // m/s




		navigation();

//		blink();


		//	if(HIL==1)
		//	{
		//		GPS.gps_air_speed = ((int)hil_buff[18]*256 + hil_buff[19])*0.01;    // m/s
		//	}

		//	if(ublox.gps_speed>1)
		//	{
		//		GPS.gps_acc = GPS.gps_acc*0.7 + 0.3*(GPS.gps_speed - old_speed)/nav_dt;
		//	}
		//	else
		//		hillgps.gps_acc = 0;
		//	old_speed = GPS.gps_speed;


	}

}

void calculate_hil_imu(){
	uint8_t i=0;
	static int hil_data[HIL_DATA_SIZE];

	for(i=1; i<(hil_packet_size-2)/2; i++)
				hil_data[i-1] = 256*hil_buff[2*i] + hil_buff[2*i+1];

	imu.roll = (float)((signed int)hil_data[0] - 30000)/100;
	imu.pitch = (float)((signed int)hil_data[1] - 30000)/100;
	imu.yaw = (float)((signed int)hil_data[2] - 30000)/100;

	imu.rollrate = -(float)((signed int)hil_data[3] - 30000)/10;
	imu.pitchrate = -(float)((signed int)hil_data[4] - 30000)/10;
	imu.yawrate = -(float)((signed int)hil_data[5] - 30000)/10;


}
