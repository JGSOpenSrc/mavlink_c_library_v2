// MESSAGE DISTANCE_SENSOR_FILTERED PACKING

#define MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED 59

MAVPACKED(
typedef struct __mavlink_distance_sensor_filtered_t {
 float current_distance; /*<  Current distance reading (in m)*/
 float covariance; /*<  Measurement covariance (in m), 0 for unknown / invalid readings */
 uint32_t time_boot_ms; /*<  Time since system boot in milliseconds */
}) mavlink_distance_sensor_filtered_t;

#define MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN 12
#define MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_MIN_LEN 12
#define MAVLINK_MSG_ID_59_LEN 12
#define MAVLINK_MSG_ID_59_MIN_LEN 12

#define MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_CRC 212
#define MAVLINK_MSG_ID_59_CRC 212



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR_FILTERED { \
	59, \
	"DISTANCE_SENSOR_FILTERED", \
	3, \
	{  { "current_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_distance_sensor_filtered_t, current_distance) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_distance_sensor_filtered_t, covariance) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_distance_sensor_filtered_t, time_boot_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR_FILTERED { \
	"DISTANCE_SENSOR_FILTERED", \
	3, \
	{  { "current_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_distance_sensor_filtered_t, current_distance) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_distance_sensor_filtered_t, covariance) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_distance_sensor_filtered_t, time_boot_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a distance_sensor_filtered message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param current_distance  Current distance reading (in m)
 * @param covariance  Measurement covariance (in m), 0 for unknown / invalid readings 
 * @param time_boot_ms  Time since system boot in milliseconds 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_distance_sensor_filtered_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float current_distance, float covariance, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN];
	_mav_put_float(buf, 0, current_distance);
	_mav_put_float(buf, 4, covariance);
	_mav_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN);
#else
	mavlink_distance_sensor_filtered_t packet;
	packet.current_distance = current_distance;
	packet.covariance = covariance;
	packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_CRC);
}

/**
 * @brief Pack a distance_sensor_filtered message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current_distance  Current distance reading (in m)
 * @param covariance  Measurement covariance (in m), 0 for unknown / invalid readings 
 * @param time_boot_ms  Time since system boot in milliseconds 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_distance_sensor_filtered_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float current_distance,float covariance,uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN];
	_mav_put_float(buf, 0, current_distance);
	_mav_put_float(buf, 4, covariance);
	_mav_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN);
#else
	mavlink_distance_sensor_filtered_t packet;
	packet.current_distance = current_distance;
	packet.covariance = covariance;
	packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_CRC);
}

/**
 * @brief Encode a distance_sensor_filtered struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param distance_sensor_filtered C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_distance_sensor_filtered_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_distance_sensor_filtered_t* distance_sensor_filtered)
{
	return mavlink_msg_distance_sensor_filtered_pack(system_id, component_id, msg, distance_sensor_filtered->current_distance, distance_sensor_filtered->covariance, distance_sensor_filtered->time_boot_ms);
}

/**
 * @brief Encode a distance_sensor_filtered struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param distance_sensor_filtered C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_distance_sensor_filtered_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_distance_sensor_filtered_t* distance_sensor_filtered)
{
	return mavlink_msg_distance_sensor_filtered_pack_chan(system_id, component_id, chan, msg, distance_sensor_filtered->current_distance, distance_sensor_filtered->covariance, distance_sensor_filtered->time_boot_ms);
}

/**
 * @brief Send a distance_sensor_filtered message
 * @param chan MAVLink channel to send the message
 *
 * @param current_distance  Current distance reading (in m)
 * @param covariance  Measurement covariance (in m), 0 for unknown / invalid readings 
 * @param time_boot_ms  Time since system boot in milliseconds 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_distance_sensor_filtered_send(mavlink_channel_t chan, float current_distance, float covariance, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN];
	_mav_put_float(buf, 0, current_distance);
	_mav_put_float(buf, 4, covariance);
	_mav_put_uint32_t(buf, 8, time_boot_ms);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED, buf, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_CRC);
#else
	mavlink_distance_sensor_filtered_t packet;
	packet.current_distance = current_distance;
	packet.covariance = covariance;
	packet.time_boot_ms = time_boot_ms;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED, (const char *)&packet, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_CRC);
#endif
}

/**
 * @brief Send a distance_sensor_filtered message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_distance_sensor_filtered_send_struct(mavlink_channel_t chan, const mavlink_distance_sensor_filtered_t* distance_sensor_filtered)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_distance_sensor_filtered_send(chan, distance_sensor_filtered->current_distance, distance_sensor_filtered->covariance, distance_sensor_filtered->time_boot_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED, (const char *)distance_sensor_filtered, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_CRC);
#endif
}

#if MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_distance_sensor_filtered_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float current_distance, float covariance, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, current_distance);
	_mav_put_float(buf, 4, covariance);
	_mav_put_uint32_t(buf, 8, time_boot_ms);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED, buf, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_CRC);
#else
	mavlink_distance_sensor_filtered_t *packet = (mavlink_distance_sensor_filtered_t *)msgbuf;
	packet->current_distance = current_distance;
	packet->covariance = covariance;
	packet->time_boot_ms = time_boot_ms;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED, (const char *)packet, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_CRC);
#endif
}
#endif

#endif

// MESSAGE DISTANCE_SENSOR_FILTERED UNPACKING


/**
 * @brief Get field current_distance from distance_sensor_filtered message
 *
 * @return  Current distance reading (in m)
 */
static inline float mavlink_msg_distance_sensor_filtered_get_current_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field covariance from distance_sensor_filtered message
 *
 * @return  Measurement covariance (in m), 0 for unknown / invalid readings 
 */
static inline float mavlink_msg_distance_sensor_filtered_get_covariance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field time_boot_ms from distance_sensor_filtered message
 *
 * @return  Time since system boot in milliseconds 
 */
static inline uint32_t mavlink_msg_distance_sensor_filtered_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a distance_sensor_filtered message into a struct
 *
 * @param msg The message to decode
 * @param distance_sensor_filtered C-struct to decode the message contents into
 */
static inline void mavlink_msg_distance_sensor_filtered_decode(const mavlink_message_t* msg, mavlink_distance_sensor_filtered_t* distance_sensor_filtered)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	distance_sensor_filtered->current_distance = mavlink_msg_distance_sensor_filtered_get_current_distance(msg);
	distance_sensor_filtered->covariance = mavlink_msg_distance_sensor_filtered_get_covariance(msg);
	distance_sensor_filtered->time_boot_ms = mavlink_msg_distance_sensor_filtered_get_time_boot_ms(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN? msg->len : MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN;
        memset(distance_sensor_filtered, 0, MAVLINK_MSG_ID_DISTANCE_SENSOR_FILTERED_LEN);
	memcpy(distance_sensor_filtered, _MAV_PAYLOAD(msg), len);
#endif
}
