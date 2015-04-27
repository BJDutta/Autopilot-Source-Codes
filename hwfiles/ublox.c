#include "ublox.h"

#ifndef NULL
#define NULL 0
#endif

gps GPS;

// Packet checksum accumulators
uint8_t         _ck_a;
uint8_t         _ck_b;

// Packet checksum accumulators
uint8_t         ck_a;
uint8_t         ck_b;

// State machine state
uint8_t         _step;
uint8_t         _msg_id;
uint16_t        _payload_length;
uint16_t        _payload_counter;

// 8 bit count of fix messages processed, used for periodic
// processing
uint8_t			_fix_count;

uint8_t         _class;

// do we have new position information?
Bool            _new_position;

// do we have new speed information?
Bool            _new_speed;

// used to update fix between status and position packets
Bool        next_fix;

uint8_t         _disable_counter;

GPS_Engine_Setting _nav_setting = GPS_ENGINE_AIRBORNE_1G; // 1G for quadcopter and 4G for Fixed wing


// Receive buffer
union {
	ubx_nav_posllh posllh;
	ubx_nav_status status;
	ubx_nav_solution solution;
	ubx_nav_velned velned;
	ubx_cfg_nav_settings nav_settings;
	uint8_t bytes[60];
} _buffer;
/*
  detect a Ublox GPS. Adds one byte, and returns true if the stream
  matches a UBlox
 */
Bool UBLOX_detect(uint8_t data)
{
	static uint8_t payload_length, payload_counter;
	static uint8_t step;
	static uint8_t ck_a, ck_b;

	switch (step) {
	case 1:
		if (PREAMBLE2 == data) {
			step++;
			break;
		}
		step = 0;
	case 0:
		if (PREAMBLE1 == data)
			step++;
		break;
	case 2:
		step++;
		ck_b = ck_a = data;
		break;
	case 3:
		step++;
		ck_b += (ck_a += data);
		break;
	case 4:
		step++;
		ck_b += (ck_a += data);
		payload_length = data;
		break;
	case 5:
		step++;
		ck_b += (ck_a += data);
		payload_counter = 0;
		break;
	case 6:
		ck_b += (ck_a += data);
		if (++payload_counter == payload_length)
			step++;
		break;
	case 7:
		step++;
		if (ck_a != data) {
			step = 0;
		}
		break;
	case 8:
		step = 0;
		if (ck_b == data) {
			// a valid UBlox packet
			return TRUE;
		}
	}
	return FALSE;
}

Bool UBLOX_read(uint8_t data)
{
//	uint8_t data;
	Bool parsed = FALSE;
//	int16_t i=0;

//	for (i = 0; i < numc; i++) {        // Process bytes received

		// read the next byte
//		data = gps_get_char();  // put getchar();

		switch(_step) {

		// Message preamble detection
		//
		// If we fail to match any of the expected bytes, we reset
		// the state machine and re-consider the failed byte as
		// the first byte of the preamble.  This improves our
		// chances of recovering from a mismatch and makes it less
		// likely that we will be fooled by the preamble appearing
		// as data in some other message.
		//
		case 1:
			if (PREAMBLE2 == data) {
				_step++;
				break;
			}
			_step = 0;
			//            Debug("reset %u", __LINE__);
			// FALLTHROUGH
		case 0:
			if(PREAMBLE1 == data)
				_step++;
			break;

			// Message header processing
			//
			// We sniff the class and message ID to decide whether we
			// are going to gather the message bytes or just discard
			// them.
			//
			// We always collect the length so that we can avoid being
			// fooled by preamble bytes in messages.
			//
		case 2:
			_step++;
			_class = data;
			_ck_b = _ck_a = data;                               // reset the checksum accumulators
			break;
		case 3:
			_step++;
			_ck_b += (_ck_a += data);                   // checksum byte
			_msg_id = data;
			break;
		case 4:
			_step++;
			_ck_b += (_ck_a += data);                   // checksum byte
			_payload_length = data;                             // payload length low byte
			break;
		case 5:
			_step++;
			_ck_b += (_ck_a += data);                   // checksum byte

			_payload_length += (uint16_t)(data<<8);
			if (_payload_length > 512) {
				//                Debug("large payload %u", (unsigned)_payload_length);
				// assume very large payloads are line noise
				_payload_length = 0;
				_step = 0;
			}
			_payload_counter = 0;                               // prepare to receive payload
			break;

			// Receive message data
			//
		case 6:
			_ck_b += (_ck_a += data);                   // checksum byte
			if (_payload_counter < sizeof(_buffer)) {
				_buffer.bytes[_payload_counter] = data;
			}
			if (++_payload_counter == _payload_length)
				_step++;
			break;

			// Checksum and message processing
			//
		case 7:
			_step++;
			if (_ck_a != data) {
				//                Debug("bad cka %x should be %x", data, _ck_a);
				_step = 0;                                              // bad checksum
			}
			break;
		case 8:
			_step = 0;
			if (_ck_b != data) {
				//                Debug("bad ckb %x should be %x", data, _ck_b);
				break;                                                  // bad checksum
			}

			if (parse_gps()) {
				parsed = TRUE;
			}
		}
//	}
	return parsed;
}

Bool parse_gps(void)
{
	if (_class == CLASS_ACK)
	{
		return FALSE;
	}
		if (_class == CLASS_CFG && _msg_id == MSG_CFG_NAV_SETTINGS) {
			if (_nav_setting != GPS_ENGINE_NONE &&
					_buffer.nav_settings.dynModel != _nav_setting) {
				// we've received the current nav settings, change the engine
				// settings and send them back
				//            Debug("Changing engine setting from %u to %u\n",
				//                  (unsigned)_buffer.nav_settings.dynModel, (unsigned)_nav_setting);
				_buffer.nav_settings.dynModel = _nav_setting;
				_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS,
						&_buffer.nav_settings,
						sizeof(_buffer.nav_settings));
			}
			return FALSE;
		}

		if (_class != CLASS_NAV) {
			//        Debug("Unexpected message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
			if (++_disable_counter == 0) {
				// disable future sends of this message id, but
				// only do this every 256 messages, as some
				// message types can't be disabled and we don't
				// want to get into an ack war
				//            Debug("Disabling message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
				_configure_message_rate(_class, _msg_id, 0);
			}
			return FALSE;
		}

	switch (_msg_id) {
	case MSG_POSLLH:
		//        Debug("MSG_POSLLH next_fix=%u", next_fix);
		GPS.time            = _buffer.posllh.time;
		GPS.longitude       = _buffer.posllh.longitude;
		GPS.latitude        = _buffer.posllh.latitude;
		GPS.altitude        = _buffer.posllh.altitude_msl / 10;
		GPS.hdop                = _buffer.posllh.horizontal_accuracy;
		GPS.fix                     = next_fix;
		if(!GPS.fix) GPS.hdop = 65535;
		_new_position = TRUE;
		break;
	case MSG_STATUS:
		//        Debug("MSG_STATUS fix_status=%u fix_type=%u",
		//              _buffer.status.fix_status,
		//              _buffer.status.fix_type);
		next_fix        = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
		if (!next_fix) {
			GPS.fix = FALSE;
		}
		break;
	case MSG_SOL:
		//        Debug("MSG_SOL fix_status=%u fix_type=%u",
		//              _buffer.solution.fix_status,
		//              _buffer.solution.fix_type);
		next_fix        = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
		if (!next_fix) {
			GPS.fix = FALSE;
		}
		GPS.num_sats        = _buffer.solution.satellites;
//		GPS.hdop            = _buffer.solution.position_DOP;
		break;
	case MSG_VELNED:
		//        Debug("MSG_VELNED");
		GPS.speed_3d        = _buffer.velned.speed_3d;                              // cm/s
		GPS.ground_speed = _buffer.velned.speed_2d;                         // cm/s
		GPS.ground_course = _buffer.velned.heading_2d / 1000;       // Heading 2D deg * 100000 rescaled to deg * 100
		GPS._have_raw_velocity = TRUE;
		GPS._vel_north  = _buffer.velned.ned_north;
		GPS._vel_east   = _buffer.velned.ned_east;
		GPS._vel_down   = _buffer.velned.ned_down;
		_new_speed = TRUE;
		break;
	default:
		//        Debug("Unexpected NAV message 0x%02x", (unsigned)_msg_id);
		if (++_disable_counter == 0) {
			//            Debug("Disabling NAV message 0x%02x", (unsigned)_msg_id);
						_configure_message_rate(CLASS_NAV, _msg_id, 0);
		}
		return FALSE;
	}

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (_new_position && _new_speed) {
		_new_speed = _new_position = FALSE;
		_fix_count++;
				if (_fix_count == 100) {
					// ask for nav settings every 20 seconds
		//			Debug("Asking for engine setting\n");
					_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
				}
		return TRUE;
	}
	return FALSE;
}

void _update_checksum(uint8_t *data, uint8_t len)// uint8_t ck_a, uint8_t ck_b)
{
	while (len--) {
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}

void _send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size)
{
	ubx_header header;
	ck_a=0, ck_b=0;
	header.preamble1 = PREAMBLE1;
	header.preamble2 = PREAMBLE2;
	header.msg_class = msg_class;
	header.msg_id    = msg_id;
	header.length    = size;

	_update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2);//, ck_a, ck_b);
	_update_checksum((uint8_t *)msg, size);//, ck_a, ck_b);

	//    _port->write((const uint8_t *)&header, sizeof(header));
	//    _port->write((const uint8_t *)msg, size);
	//    _port->write((const uint8_t *)&ck_a, 1);
	//    _port->write((const uint8_t *)&ck_b, 1);

	gps_puts((uint8_t *)&header, sizeof(header));
	gps_puts((uint8_t *)msg, size);
	gps_puts((uint8_t *)&ck_a, 1);
	gps_puts((uint8_t *)&ck_b, 1);
}


void _configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
	ubx_cfg_msg_rate msg;
	msg.msg_class = msg_class;
	msg.msg_id    = msg_id;
	msg.rate          = rate;
	_send_message(CLASS_CFG, MSG_CFG_SET_RATE, &msg, sizeof(msg));

//	delay(100);
}


void _configure_gps(void)
{
	ubx_cfg_nav_rate msg;
	//	const unsigned baudrates[4] = {9600U, 19200U, 38400U, 57600U};
	//	FastSerial *_fs = (FastSerial *)_port;

	//    // the GPS may be setup for a different baud rate. This ensures
	//    // it gets configured correctly
	//    for (uint8_t i=0; i<4; i++) {
	//        _fs->begin(baudrates[i]);
	//        _write_progstr_block(_fs, _ublox_set_binary, _ublox_set_binary_size);
	//        while (_fs->tx_pending()) delay(1);
	//    }
	//    _fs->begin(38400U);

	GPS.hdop = 65535;  // so that horizontal accuracy is worst if GPS is not connected

	// ask for navigation solutions every 200ms
	msg.measure_rate_ms = 200;
	msg.nav_rate        = 1;
	msg.timeref         = 0;     // UTC time
	_send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));delay(100);


	// ask for the messages we parse to be sent on every navigation solution
	_configure_message_rate(CLASS_NAV, MSG_POSLLH, 1);	delay(100);
	_configure_message_rate(CLASS_NAV, MSG_STATUS, 1);	delay(100);
	_configure_message_rate(CLASS_NAV, MSG_SOL, 1);		delay(100);
	_configure_message_rate(CLASS_NAV, MSG_VELNED, 1);	delay(100);

	//    // ask for the current navigation settings
	//	Debug("Asking for engine setting\n");
	    _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);delay(100);
}
