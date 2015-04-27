#include "waypoint.h"
#include "gps.h"
#include "mlink.h"
#include "param.h"
#include "i2ceeprom.h"
#include "lpc_types.h"
#include "imu.h"
#include "control.h"

char isVideo =0; // if video is to be shot
char isVideoTest =0; // if video  testclick is to be shot
char isPhotoTest =0; // if photo  testclick is to be shot


void waypoint_init(){

	eeprom_read_mid(WP.eeprom, (uint16_t)(*MISC_MISSION_SIZE+5)*MAVLINK_MSG_ID_MISSION_ITEM_LEN);

}

void waypoint_save(){

	eeprom_write_mid(WP.eeprom, (uint16_t)(*MISC_MISSION_SIZE+5)*MAVLINK_MSG_ID_MISSION_ITEM_LEN);

}


void nav_generate(float type){

	float WX, WY, PX, PY, psi, cospsi, sinpsi;
	int i=0, col=0, row=0, dir=1, index=1;

	if(*FIELD_GEN==5){
		isVideo=1;
	}else isVideo=0;

	*FIELD_GEN = 0;

	switch((int)(type)){		// mission type
	case 1:{			// Grip mapping mission
		*MISC_MISSION_SIZE = (uint16_t)(*FIELD_GRIDX)*(*FIELD_GRIDY)+1;


		WP.item[0].x = home.X/10000000.0f; WP.item[0].y = home.Y/10000000.0f; WP.item[0].z = (*FIELD_ALT);  WP.item[0].param2 = (*FIELD_WPRADIUS); WP.item[0].param1 = (2); WP.item[0].param4 = imu.yaw; ////need to check setting the home as first WP

		WP.item[0].seq = 0; ///< Sequence
		WP.item[0].command = MAV_CMD_NAV_WAYPOINT; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		WP.item[0].target_system = 13; ///< System ID
		WP.item[0].target_component = 190; ///< Component ID
		WP.item[0].frame=0; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		WP.item[0].current = 0; ///< false:0, true:1
		WP.item[0].autocontinue = 1; ///< autocontinue to next wp

		psi = imu.yaw;

		cospsi = cos(psi*D2R);
		sinpsi = sin(psi*D2R);

		for(col=0; col<(*FIELD_GRIDX); col++){


			WX = ((*FIELD_LENGTH)/(*FIELD_GRIDX))*((float)col + 0.5 - (*FIELD_GRIDX)/2);

			for(i=0; i<(*FIELD_GRIDY); i++){

				WY = ((*FIELD_BREADTH)/(*FIELD_GRIDY))*((float)row + dir*0.5);

				row = row + dir;


				// rotate the field frame by yaw angle;

				PX =  WX*cospsi  +   WY*sinpsi;
				PY = -WX*sinpsi  +   WY*cospsi;


				WP.item[index].x = ((PY/Re)*ToDeg) + home.X/10000000.0f;

				WP.item[index].y = ((PX/(Re*cos(ToRad*(float)(home.X)/10000000)))*ToDeg) + home.Y/10000000.0f;

				WP.item[index].z = (*FIELD_ALT);

				WP.item[index].param2 = (*FIELD_WPRADIUS); // tolerance radius

				WP.item[index].param1 = (*FIELD_WPTIME);

				WP.item[index].param3 = 0;

				WP.item[index].param4 = imu.yaw;  // target heading, not implemented in control system yet


				WP.item[index].seq = index; ///< Sequence
				WP.item[index].command = MAV_CMD_NAV_WAYPOINT; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
				WP.item[index].target_system = 13; ///< System ID
				WP.item[index].target_component = 190; ///< Component ID
				WP.item[index].frame=0; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
				WP.item[index].current = 0; ///< false:0, true:1
				WP.item[index].autocontinue = 1; ///< autocontinue to next wp

				index++;

			}

			if(dir==1)dir=-1;   // toggle travel direction along y axis
			else dir = 1;
		}
	}
	break;
	case 2:					// altitude mission
	{
		*MISC_MISSION_SIZE = 1;


		WP.item[0].x = home.X/10000000.0f; WP.item[0].y = home.Y/10000000.0f; WP.item[0].z = (*MISC_HOVALT);  WP.item[0].param2 = (*FIELD_WPRADIUS); WP.item[0].param1 = (5); WP.item[0].param4 = imu.yaw; // setting the home as first WP

		WP.item[0].seq = 0; ///< Sequence
		WP.item[0].command = MAV_CMD_NAV_WAYPOINT; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		WP.item[0].target_system = 13; ///< System ID
		WP.item[0].target_component = 190; ///< Component ID
		WP.item[0].frame=0; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		WP.item[0].current = 0; ///< false:0, true:1
		WP.item[0].autocontinue = 1; ///< autocontinue to next wp

	}
	break;
	case 3:					// test flight
	{
		*MISC_MISSION_SIZE = 6;


		WP.item[0].x = home.X/10000000.0f; WP.item[0].y = home.Y/10000000.0f; WP.item[0].z = (*MISC_NAVALT);  WP.item[0].param2 = 6; WP.item[0].param1 = 0.1; WP.item[0].param4 = imu.yaw; // setting the home as first WP

		WP.item[5].x = home.X/10000000.0f; WP.item[5].y = home.Y/10000000.0f; WP.item[5].z = (*MISC_NAVALT);  WP.item[5].param2 = 6; WP.item[5].param1 = 0.1; WP.item[5].param4 = imu.yaw; // setting the home as last WP

		WP.item[5].seq = 5; WP.item[0].seq = 0; ///< Sequence
		WP.item[5].command = WP.item[0].command = MAV_CMD_NAV_WAYPOINT; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		WP.item[5].target_system = WP.item[0].target_system = 13; ///< System ID
		WP.item[5].target_component = WP.item[0].target_component = 190; ///< Component ID
		WP.item[5].frame = WP.item[0].frame=0; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		WP.item[5].current = WP.item[0].current = 0; ///< false:0, true:1
		WP.item[5].autocontinue = WP.item[0].autocontinue = 1; ///< autocontinue to next wp


		psi = imu.yaw;

		cospsi = cos(psi*D2R);
		sinpsi = sin(psi*D2R);

		for(col=0; col<(2); col++){  // grid size 2x2 of 15m x 15m


			WX = ((15)/(2))*((float)col + 0.5 - (2)/2);

			for(i=0; i<(2); i++){

				WY = ((15)/(2))*((float)row + dir*0.5);

				row = row + dir;


				// rotate the field frame by yaw angle;

				PX =  WX*cospsi  +   WY*sinpsi;
				PY = -WX*sinpsi  +   WY*cospsi;


				WP.item[index].x = ((PY/Re)*ToDeg) + home.X/10000000.0f;

				WP.item[index].y = ((PX/(Re*cos(ToRad*(float)(home.X)/10000000)))*ToDeg) + home.Y/10000000.0f;

				WP.item[index].z = (*MISC_NAVALT);

				WP.item[index].param2 = 6; // tolerance radius

				WP.item[index].param1 = 0.1;

				WP.item[index].param3 = 0;

				WP.item[index].param4 = imu.yaw;  // target heading, not implemented in control system yet


				WP.item[index].seq = index; ///< Sequence
				WP.item[index].command = MAV_CMD_NAV_WAYPOINT; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
				WP.item[index].target_system = 13; ///< System ID
				WP.item[index].target_component = 190; ///< Component ID
				WP.item[index].frame=0; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
				WP.item[index].current = 0; ///< false:0, true:1
				WP.item[index].autocontinue = 1; ///< autocontinue to next wp

				index++;

			}

			if(dir==1)dir=-1;   // toggle travel direction along y axis
			else dir = 1;
		}
	}
	break;
	case 4:			// for photo click
	case 5:			// for video click
	{
		*MISC_MISSION_SIZE = 3;

		float brng;

//		float lat1 = ((home.lat)/10000000.0f) *D2R;
//		float lat2 = *FIELD_LENGTH *D2R;
//		float dLat = (*FIELD_LENGTH - home.lat/10000000.0f) *D2R;
//		float dLon = (*FIELD_BREADTH - home.lon/10000000.0f) *D2R;
//
//		float y = sin(dLon) * cos(lat2);
//		float x = cos(lat1) * sin(lat2) -
//				sin(lat1) * cos(lat2) * cos(dLon);
//
//		float a = sin(dLat/2) * sin(dLat/2) +  sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
//		float c = 2 * atan2(sqrt(a), sqrt(1-a));
//		float TargetDistance = (int)(Re * c);

//		if(TargetDistance>25){
//			brng = (atan2(y, x))*R2D;
//		}
//		else

			brng = imu.yaw;

		WP.item[0].x = home.X/10000000.0f; WP.item[0].y = home.Y/10000000.0f; WP.item[0].z = (*MISC_HOVALT);  WP.item[0].param2 = (*FIELD_WPRADIUS); WP.item[0].param1 = (.01); WP.item[0].param4 = brng; // setting the home as first WP

		WP.item[0].seq = 0; ///< Sequence
		WP.item[0].command = MAV_CMD_NAV_WAYPOINT; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		WP.item[0].target_system = 13; ///< System ID
		WP.item[0].target_component = 190; ///< Component ID
		WP.item[0].frame=0; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		WP.item[0].current = 0; ///< false:0, true:1
		WP.item[0].autocontinue = 1; ///< autocontinue to next wp


		WP.item[1].x = *FIELD_LENGTH; WP.item[1].y = *FIELD_BREADTH; WP.item[1].z = (*MISC_HOVALT);  WP.item[1].param2 = (*FIELD_WPRADIUS); WP.item[1].param1 = (.01); WP.item[1].param4 = brng; //

		WP.item[1].seq = 1; ///< Sequence
		WP.item[1].command = MAV_CMD_NAV_WAYPOINT; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		WP.item[1].target_system = 13; ///< System ID
		WP.item[1].target_component = 190; ///< Component ID
		WP.item[1].frame=0; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		WP.item[1].current = 0; ///< false:0, true:1
		WP.item[1].autocontinue = 1; ///< autocontinue to next wp

		WP.item[2].x = *FIELD_LENGTH; WP.item[2].y = *FIELD_BREADTH; WP.item[2].z = (*MISC_HOVALT);  WP.item[2].param2 = (*FIELD_WPRADIUS);  WP.item[2].param4 = imu.yaw; //

		if (isVideo)WP.item[2].param1 = *FIELD_WPTIME; // video for WPtime
		else WP.item[2].param1 = (5);  //photo for 5 secs

		WP.item[2].seq = 2; ///< Sequence
		WP.item[2].command = MAV_CMD_NAV_WAYPOINT; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		WP.item[2].target_system = 13; ///< System ID
		WP.item[2].target_component = 190; ///< Component ID
		WP.item[2].frame=0; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		WP.item[2].current = 0; ///< false:0, true:1
		WP.item[2].autocontinue = 1; ///< autocontinue to next wp

	}
	break;
	default: break;
	}



	waypoint_save();
	eeprom_write(global_data.eeprom, sizeof(global_data.eeprom));
}
