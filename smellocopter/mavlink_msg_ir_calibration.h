// MESSAGE IR_CALIBRATION PACKING

#define MAVLINK_MSG_ID_IR_CALIBRATION 58

MAVPACKED(
typedef struct __mavlink_ir_calibration_t {
 float data; /*<  Variable for exchanging sensor samples or estimator coefficients between GCS and FMU.*/
 uint8_t data_code; /*<  Code from IR_CAL_CODE_ENUM that puts the data into context.*/
}) mavlink_ir_calibration_t;

#define MAVLINK_MSG_ID_IR_CALIBRATION_LEN 5
#define MAVLINK_MSG_ID_IR_CALIBRATION_MIN_LEN 5
#define MAVLINK_MSG_ID_58_LEN 5
#define MAVLINK_MSG_ID_58_MIN_LEN 5

#define MAVLINK_MSG_ID_IR_CALIBRATION_CRC 127
#define MAVLINK_MSG_ID_58_CRC 127



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IR_CALIBRATION { \
	58, \
	"IR_CALIBRATION", \
	2, \
	{  { "data", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ir_calibration_t, data) }, \
         { "data_code", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_ir_calibration_t, data_code) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IR_CALIBRATION { \
	"IR_CALIBRATION", \
	2, \
	{  { "data", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ir_calibration_t, data) }, \
         { "data_code", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_ir_calibration_t, data_code) }, \
         } \
}
#endif

/**
 * @brief Pack a ir_calibration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param data_code  Code from IR_CAL_CODE_ENUM that puts the data into context.
 * @param data  Variable for exchanging sensor samples or estimator coefficients between GCS and FMU.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ir_calibration_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t data_code, float data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IR_CALIBRATION_LEN];
	_mav_put_float(buf, 0, data);
	_mav_put_uint8_t(buf, 4, data_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IR_CALIBRATION_LEN);
#else
	mavlink_ir_calibration_t packet;
	packet.data = data;
	packet.data_code = data_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IR_CALIBRATION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IR_CALIBRATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IR_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_CRC);
}

/**
 * @brief Pack a ir_calibration message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data_code  Code from IR_CAL_CODE_ENUM that puts the data into context.
 * @param data  Variable for exchanging sensor samples or estimator coefficients between GCS and FMU.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ir_calibration_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t data_code,float data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IR_CALIBRATION_LEN];
	_mav_put_float(buf, 0, data);
	_mav_put_uint8_t(buf, 4, data_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IR_CALIBRATION_LEN);
#else
	mavlink_ir_calibration_t packet;
	packet.data = data;
	packet.data_code = data_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IR_CALIBRATION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IR_CALIBRATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IR_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_CRC);
}

/**
 * @brief Encode a ir_calibration struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ir_calibration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ir_calibration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ir_calibration_t* ir_calibration)
{
	return mavlink_msg_ir_calibration_pack(system_id, component_id, msg, ir_calibration->data_code, ir_calibration->data);
}

/**
 * @brief Encode a ir_calibration struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ir_calibration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ir_calibration_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ir_calibration_t* ir_calibration)
{
	return mavlink_msg_ir_calibration_pack_chan(system_id, component_id, chan, msg, ir_calibration->data_code, ir_calibration->data);
}

/**
 * @brief Send a ir_calibration message
 * @param chan MAVLink channel to send the message
 *
 * @param data_code  Code from IR_CAL_CODE_ENUM that puts the data into context.
 * @param data  Variable for exchanging sensor samples or estimator coefficients between GCS and FMU.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ir_calibration_send(mavlink_channel_t chan, uint8_t data_code, float data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IR_CALIBRATION_LEN];
	_mav_put_float(buf, 0, data);
	_mav_put_uint8_t(buf, 4, data_code);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_CALIBRATION, buf, MAVLINK_MSG_ID_IR_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_CRC);
#else
	mavlink_ir_calibration_t packet;
	packet.data = data;
	packet.data_code = data_code;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_CALIBRATION, (const char *)&packet, MAVLINK_MSG_ID_IR_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_CRC);
#endif
}

/**
 * @brief Send a ir_calibration message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ir_calibration_send_struct(mavlink_channel_t chan, const mavlink_ir_calibration_t* ir_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ir_calibration_send(chan, ir_calibration->data_code, ir_calibration->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_CALIBRATION, (const char *)ir_calibration, MAVLINK_MSG_ID_IR_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_IR_CALIBRATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ir_calibration_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t data_code, float data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, data);
	_mav_put_uint8_t(buf, 4, data_code);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_CALIBRATION, buf, MAVLINK_MSG_ID_IR_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_CRC);
#else
	mavlink_ir_calibration_t *packet = (mavlink_ir_calibration_t *)msgbuf;
	packet->data = data;
	packet->data_code = data_code;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_CALIBRATION, (const char *)packet, MAVLINK_MSG_ID_IR_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_LEN, MAVLINK_MSG_ID_IR_CALIBRATION_CRC);
#endif
}
#endif

#endif

// MESSAGE IR_CALIBRATION UNPACKING


/**
 * @brief Get field data_code from ir_calibration message
 *
 * @return  Code from IR_CAL_CODE_ENUM that puts the data into context.
 */
static inline uint8_t mavlink_msg_ir_calibration_get_data_code(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field data from ir_calibration message
 *
 * @return  Variable for exchanging sensor samples or estimator coefficients between GCS and FMU.
 */
static inline float mavlink_msg_ir_calibration_get_data(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a ir_calibration message into a struct
 *
 * @param msg The message to decode
 * @param ir_calibration C-struct to decode the message contents into
 */
static inline void mavlink_msg_ir_calibration_decode(const mavlink_message_t* msg, mavlink_ir_calibration_t* ir_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	ir_calibration->data = mavlink_msg_ir_calibration_get_data(msg);
	ir_calibration->data_code = mavlink_msg_ir_calibration_get_data_code(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IR_CALIBRATION_LEN? msg->len : MAVLINK_MSG_ID_IR_CALIBRATION_LEN;
        memset(ir_calibration, 0, MAVLINK_MSG_ID_IR_CALIBRATION_LEN);
	memcpy(ir_calibration, _MAV_PAYLOAD(msg), len);
#endif
}
