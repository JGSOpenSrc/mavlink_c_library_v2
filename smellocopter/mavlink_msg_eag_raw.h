// MESSAGE EAG_RAW PACKING

#define MAVLINK_MSG_ID_EAG_RAW 57

MAVPACKED(
typedef struct __mavlink_eag_raw_t {
 uint32_t time_boot_ms; /*<  Time since system boot in milliseconds */
 uint8_t raw_data; /*<  Raw integer output from EAG ADC */
}) mavlink_eag_raw_t;

#define MAVLINK_MSG_ID_EAG_RAW_LEN 5
#define MAVLINK_MSG_ID_EAG_RAW_MIN_LEN 5
#define MAVLINK_MSG_ID_57_LEN 5
#define MAVLINK_MSG_ID_57_MIN_LEN 5

#define MAVLINK_MSG_ID_EAG_RAW_CRC 111
#define MAVLINK_MSG_ID_57_CRC 111



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EAG_RAW { \
	57, \
	"EAG_RAW", \
	2, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_eag_raw_t, time_boot_ms) }, \
         { "raw_data", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_eag_raw_t, raw_data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EAG_RAW { \
	"EAG_RAW", \
	2, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_eag_raw_t, time_boot_ms) }, \
         { "raw_data", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_eag_raw_t, raw_data) }, \
         } \
}
#endif

/**
 * @brief Pack a eag_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param raw_data  Raw integer output from EAG ADC 
 * @param time_boot_ms  Time since system boot in milliseconds 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eag_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t raw_data, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EAG_RAW_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, raw_data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EAG_RAW_LEN);
#else
	mavlink_eag_raw_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.raw_data = raw_data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EAG_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EAG_RAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EAG_RAW_MIN_LEN, MAVLINK_MSG_ID_EAG_RAW_LEN, MAVLINK_MSG_ID_EAG_RAW_CRC);
}

/**
 * @brief Pack a eag_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param raw_data  Raw integer output from EAG ADC 
 * @param time_boot_ms  Time since system boot in milliseconds 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eag_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t raw_data,uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EAG_RAW_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, raw_data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EAG_RAW_LEN);
#else
	mavlink_eag_raw_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.raw_data = raw_data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EAG_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EAG_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EAG_RAW_MIN_LEN, MAVLINK_MSG_ID_EAG_RAW_LEN, MAVLINK_MSG_ID_EAG_RAW_CRC);
}

/**
 * @brief Encode a eag_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eag_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eag_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eag_raw_t* eag_raw)
{
	return mavlink_msg_eag_raw_pack(system_id, component_id, msg, eag_raw->raw_data, eag_raw->time_boot_ms);
}

/**
 * @brief Encode a eag_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eag_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eag_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eag_raw_t* eag_raw)
{
	return mavlink_msg_eag_raw_pack_chan(system_id, component_id, chan, msg, eag_raw->raw_data, eag_raw->time_boot_ms);
}

/**
 * @brief Send a eag_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param raw_data  Raw integer output from EAG ADC 
 * @param time_boot_ms  Time since system boot in milliseconds 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eag_raw_send(mavlink_channel_t chan, uint8_t raw_data, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EAG_RAW_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, raw_data);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EAG_RAW, buf, MAVLINK_MSG_ID_EAG_RAW_MIN_LEN, MAVLINK_MSG_ID_EAG_RAW_LEN, MAVLINK_MSG_ID_EAG_RAW_CRC);
#else
	mavlink_eag_raw_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.raw_data = raw_data;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EAG_RAW, (const char *)&packet, MAVLINK_MSG_ID_EAG_RAW_MIN_LEN, MAVLINK_MSG_ID_EAG_RAW_LEN, MAVLINK_MSG_ID_EAG_RAW_CRC);
#endif
}

/**
 * @brief Send a eag_raw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eag_raw_send_struct(mavlink_channel_t chan, const mavlink_eag_raw_t* eag_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eag_raw_send(chan, eag_raw->raw_data, eag_raw->time_boot_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EAG_RAW, (const char *)eag_raw, MAVLINK_MSG_ID_EAG_RAW_MIN_LEN, MAVLINK_MSG_ID_EAG_RAW_LEN, MAVLINK_MSG_ID_EAG_RAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_EAG_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eag_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t raw_data, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, raw_data);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EAG_RAW, buf, MAVLINK_MSG_ID_EAG_RAW_MIN_LEN, MAVLINK_MSG_ID_EAG_RAW_LEN, MAVLINK_MSG_ID_EAG_RAW_CRC);
#else
	mavlink_eag_raw_t *packet = (mavlink_eag_raw_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->raw_data = raw_data;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EAG_RAW, (const char *)packet, MAVLINK_MSG_ID_EAG_RAW_MIN_LEN, MAVLINK_MSG_ID_EAG_RAW_LEN, MAVLINK_MSG_ID_EAG_RAW_CRC);
#endif
}
#endif

#endif

// MESSAGE EAG_RAW UNPACKING


/**
 * @brief Get field raw_data from eag_raw message
 *
 * @return  Raw integer output from EAG ADC 
 */
static inline uint8_t mavlink_msg_eag_raw_get_raw_data(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field time_boot_ms from eag_raw message
 *
 * @return  Time since system boot in milliseconds 
 */
static inline uint32_t mavlink_msg_eag_raw_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a eag_raw message into a struct
 *
 * @param msg The message to decode
 * @param eag_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_eag_raw_decode(const mavlink_message_t* msg, mavlink_eag_raw_t* eag_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	eag_raw->time_boot_ms = mavlink_msg_eag_raw_get_time_boot_ms(msg);
	eag_raw->raw_data = mavlink_msg_eag_raw_get_raw_data(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EAG_RAW_LEN? msg->len : MAVLINK_MSG_ID_EAG_RAW_LEN;
        memset(eag_raw, 0, MAVLINK_MSG_ID_EAG_RAW_LEN);
	memcpy(eag_raw, _MAV_PAYLOAD(msg), len);
#endif
}
