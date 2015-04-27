//#include "param.h"
//
//
//
//
//static void handle_mavlink_message(mavlink_channel_t chan,
//		mavlink_message_t* msg)
//{
//	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//	uint32_t len;
//	switch (chan)
//	{
//
//	}
//}
//
//
////
////	/**
////	 * @brief This is the main loop
////	 *
////	 * It will be executed at maximum MCU speed (xx MHz)
////	 */
////	void main_loop(void)
////	{
////		// Load default eeprom parameters as fallback
////		global_data_reset_param_defaults();
////
////		while (1)
////		{
////			msleep(100); // Use any wait or periodec check function you want, better not use sleep
////
////			mavlink_message_t msg;
////			mavlink_status_t status;
////			if(mavlink_parse_char(MAVLINK_COMM_0, uart0_get_char(), &msg, &status))
////			{
////				// See onboard integration tutorial
////				handle_mavlink_message(MAVLINK_COMM_0, &msg); // If a parameter request occured,
////				// it will be processed
////			}
////
////			communication_queued_send(); // Send parameters at 10 Hz, if previously requested
////		}
////}




