#pragma once
// MESSAGE SET_YAW PACKING

#define MAVLINK_MSG_ID_SET_YAW 95

MAVPACKED(
typedef struct __mavlink_set_yaw_t {
 float Yaw; /*< yaw*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
}) mavlink_set_yaw_t;

#define MAVLINK_MSG_ID_SET_YAW_LEN 6
#define MAVLINK_MSG_ID_SET_YAW_MIN_LEN 6
#define MAVLINK_MSG_ID_95_LEN 6
#define MAVLINK_MSG_ID_95_MIN_LEN 6

#define MAVLINK_MSG_ID_SET_YAW_CRC 202
#define MAVLINK_MSG_ID_95_CRC 202



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_YAW { \
    95, \
    "SET_YAW", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_set_yaw_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_set_yaw_t, target_component) }, \
         { "Yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_yaw_t, Yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_YAW { \
    "SET_YAW", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_set_yaw_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_set_yaw_t, target_component) }, \
         { "Yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_yaw_t, Yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a set_yaw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param Yaw yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_yaw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, float Yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_YAW_LEN];
    _mav_put_float(buf, 0, Yaw);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_YAW_LEN);
#else
    mavlink_set_yaw_t packet;
    packet.Yaw = Yaw;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_YAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_YAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_YAW_MIN_LEN, MAVLINK_MSG_ID_SET_YAW_LEN, MAVLINK_MSG_ID_SET_YAW_CRC);
}

/**
 * @brief Pack a set_yaw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param Yaw yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_yaw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,float Yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_YAW_LEN];
    _mav_put_float(buf, 0, Yaw);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_YAW_LEN);
#else
    mavlink_set_yaw_t packet;
    packet.Yaw = Yaw;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_YAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_YAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_YAW_MIN_LEN, MAVLINK_MSG_ID_SET_YAW_LEN, MAVLINK_MSG_ID_SET_YAW_CRC);
}

/**
 * @brief Encode a set_yaw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_yaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_yaw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_yaw_t* set_yaw)
{
    return mavlink_msg_set_yaw_pack(system_id, component_id, msg, set_yaw->target_system, set_yaw->target_component, set_yaw->Yaw);
}

/**
 * @brief Encode a set_yaw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_yaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_yaw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_yaw_t* set_yaw)
{
    return mavlink_msg_set_yaw_pack_chan(system_id, component_id, chan, msg, set_yaw->target_system, set_yaw->target_component, set_yaw->Yaw);
}

/**
 * @brief Send a set_yaw message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param Yaw yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_yaw_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float Yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_YAW_LEN];
    _mav_put_float(buf, 0, Yaw);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_YAW, buf, MAVLINK_MSG_ID_SET_YAW_MIN_LEN, MAVLINK_MSG_ID_SET_YAW_LEN, MAVLINK_MSG_ID_SET_YAW_CRC);
#else
    mavlink_set_yaw_t packet;
    packet.Yaw = Yaw;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_YAW, (const char *)&packet, MAVLINK_MSG_ID_SET_YAW_MIN_LEN, MAVLINK_MSG_ID_SET_YAW_LEN, MAVLINK_MSG_ID_SET_YAW_CRC);
#endif
}

/**
 * @brief Send a set_yaw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_yaw_send_struct(mavlink_channel_t chan, const mavlink_set_yaw_t* set_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_yaw_send(chan, set_yaw->target_system, set_yaw->target_component, set_yaw->Yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_YAW, (const char *)set_yaw, MAVLINK_MSG_ID_SET_YAW_MIN_LEN, MAVLINK_MSG_ID_SET_YAW_LEN, MAVLINK_MSG_ID_SET_YAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_YAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_yaw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, float Yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, Yaw);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_YAW, buf, MAVLINK_MSG_ID_SET_YAW_MIN_LEN, MAVLINK_MSG_ID_SET_YAW_LEN, MAVLINK_MSG_ID_SET_YAW_CRC);
#else
    mavlink_set_yaw_t *packet = (mavlink_set_yaw_t *)msgbuf;
    packet->Yaw = Yaw;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_YAW, (const char *)packet, MAVLINK_MSG_ID_SET_YAW_MIN_LEN, MAVLINK_MSG_ID_SET_YAW_LEN, MAVLINK_MSG_ID_SET_YAW_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_YAW UNPACKING


/**
 * @brief Get field target_system from set_yaw message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_yaw_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from set_yaw message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_yaw_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field Yaw from set_yaw message
 *
 * @return yaw
 */
static inline float mavlink_msg_set_yaw_get_Yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a set_yaw message into a struct
 *
 * @param msg The message to decode
 * @param set_yaw C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_yaw_decode(const mavlink_message_t* msg, mavlink_set_yaw_t* set_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_yaw->Yaw = mavlink_msg_set_yaw_get_Yaw(msg);
    set_yaw->target_system = mavlink_msg_set_yaw_get_target_system(msg);
    set_yaw->target_component = mavlink_msg_set_yaw_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_YAW_LEN? msg->len : MAVLINK_MSG_ID_SET_YAW_LEN;
        memset(set_yaw, 0, MAVLINK_MSG_ID_SET_YAW_LEN);
    memcpy(set_yaw, _MAV_PAYLOAD(msg), len);
#endif
}
