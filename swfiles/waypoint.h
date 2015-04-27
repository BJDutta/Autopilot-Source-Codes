#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "mlink.h"



#define WP_SIZE 50

extern char isVideo, isVideoTest, isPhotoTest;

union {
	mavlink_mission_item_t item[WP_SIZE];
	char eeprom[WP_SIZE*MAVLINK_MSG_ID_MISSION_ITEM_LEN];
}WP;

void waypoint_init();
void waypoint_save();
void nav_generate(float type);




















#endif
