#ifndef UBLOX_H
#define UBLOX_H

#include "lpc_types.h"
#include "LED_Blink.h"
#include "gps.h"
#include "comm.h"

typedef enum {
        GPS_ENGINE_NONE        = -1,
        GPS_ENGINE_PORTABLE    = 0,
        GPS_ENGINE_STATIONARY  = 2,
        GPS_ENGINE_PEDESTRIAN  = 3,
        GPS_ENGINE_AUTOMOTIVE  = 4,
        GPS_ENGINE_SEA         = 5,
        GPS_ENGINE_AIRBORNE_1G = 6,
        GPS_ENGINE_AIRBORNE_2G = 7,
        GPS_ENGINE_AIRBORNE_4G = 8
}GPS_Engine_Setting;

typedef enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
}ubs_protocol_bytes;

typedef enum  {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
}ubs_nav_fix_type;

typedef enum {
    NAV_STATUS_FIX_VALID = 1
}ubx_nav_status_bits;

typedef struct  {
	uint32_t time;                                  // GPS msToW
	int32_t longitude;
	int32_t latitude;
	int32_t altitude_ellipsoid;
	int32_t altitude;
	uint8_t fix;
	uint8_t num_sats;
	uint16_t hdop;

	uint32_t speed_3d;
	uint32_t ground_speed;
	int32_t ground_course;  // centidegrees -18000 to 18000
	Bool _have_raw_velocity;
	int32_t _vel_north;
	int32_t _vel_east;
	int32_t _vel_down;

}gps;

typedef struct {
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
}ubx_header;

typedef struct {
	uint16_t measure_rate_ms;
	uint16_t nav_rate;
	uint16_t timeref;
}ubx_cfg_nav_rate ;

typedef struct {
	uint8_t msg_class;
	uint8_t msg_id;
	uint8_t rate;
}ubx_cfg_msg_rate ;

typedef struct  {
	uint16_t mask;
	uint8_t dynModel;
	uint8_t fixMode;
	int32_t fixedAlt;
	uint32_t fixedAltVar;
	int8_t minElev;
	uint8_t drLimit;
	uint16_t pDop;
	uint16_t tDop;
	uint16_t pAcc;
	uint16_t tAcc;
	uint8_t staticHoldThresh;
	uint8_t res1;
	uint32_t res2;
	uint32_t res3;
	uint32_t res4;
}ubx_cfg_nav_settings;

typedef struct  {
	uint32_t time;                                  // GPS msToW
	int32_t longitude;
	int32_t latitude;
	int32_t altitude_ellipsoid;
	int32_t altitude_msl;
	uint32_t horizontal_accuracy;
	uint32_t vertical_accuracy;
}ubx_nav_posllh;

typedef struct  {
	uint32_t time;                                  // GPS msToW
	uint8_t fix_type;
	uint8_t fix_status;
	uint8_t differential_status;
	uint8_t res;
	uint32_t time_to_first_fix;
	uint32_t uptime;                                // milliseconds
}ubx_nav_status;

typedef struct {
	uint32_t time;
	int32_t time_nsec;
	int16_t week;
	uint8_t fix_type;
	uint8_t fix_status;
	int32_t ecef_x;
	int32_t ecef_y;
	int32_t ecef_z;
	uint32_t position_accuracy_3d;
	int32_t ecef_x_velocity;
	int32_t ecef_y_velocity;
	int32_t ecef_z_velocity;
	uint32_t speed_accuracy;
	uint16_t position_DOP;
	uint8_t res;
	uint8_t satellites;
	uint32_t res2;
}ubx_nav_solution ;

typedef struct  {
	uint32_t time;                                  // GPS msToW
	int32_t ned_north;
	int32_t ned_east;
	int32_t ned_down;
	uint32_t speed_3d;
	uint32_t speed_2d;
	int32_t heading_2d;
	uint32_t speed_accuracy;
	uint32_t heading_accuracy;
}ubx_nav_velned;

extern gps GPS;

Bool UBLOX_detect(uint8_t data);
Bool UBLOX_read(uint8_t data);
Bool parse_gps(void);
void _update_checksum(uint8_t *data, uint8_t len);
void _send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size);
void _configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
void _configure_gps(void);












#endif
