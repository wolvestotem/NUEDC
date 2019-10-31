#pragma once
// MESSAGE SET_VELOCITY_BODY PACKING

#define MAVLINK_MSG_ID_SET_VELOCITY_BODY 88

MAVPACKED(
typedef struct __mavlink_set_velocity_body_t {
 uint32_t time_boot_ms; /*< Timestamp in milliseconds since system boot*/
 float vx; /*< X velocity in NED frame in meter / s*/
 float vy; /*< Y velocity in NED frame in meter / s*/
 float vz; /*< Z velocity in NED frame in meter / s*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
}) mavlink_set_velocity_body_t;

#define MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN 18
#define MAVLINK_MSG_ID_SET_VELOCITY_BODY_MIN_LEN 18
#define MAVLINK_MSG_ID_88_LEN 18
#define MAVLINK_MSG_ID_88_MIN_LEN 18

#define MAVLINK_MSG_ID_SET_VELOCITY_BODY_CRC 16
#define MAVLINK_MSG_ID_88_CRC 16



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_VELOCITY_BODY { \
    88, \
    "SET_VELOCITY_BODY", \
    6, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_velocity_body_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_velocity_body_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_set_velocity_body_t, target_component) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_velocity_body_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_velocity_body_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_velocity_body_t, vz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_VELOCITY_BODY { \
    "SET_VELOCITY_BODY", \
    6, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_velocity_body_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_velocity_body_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_set_velocity_body_t, target_component) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_velocity_body_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_velocity_body_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_velocity_body_t, vz) }, \
         } \
}
#endif

/**
 * @brief Pack a set_velocity_body message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_velocity_body_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, vz);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN);
#else
    mavlink_set_velocity_body_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_VELOCITY_BODY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_VELOCITY_BODY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_CRC);
}

/**
 * @brief Pack a set_velocity_body message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_velocity_body_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,float vx,float vy,float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, vz);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN);
#else
    mavlink_set_velocity_body_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_VELOCITY_BODY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_VELOCITY_BODY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_CRC);
}

/**
 * @brief Encode a set_velocity_body struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_velocity_body C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_velocity_body_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_velocity_body_t* set_velocity_body)
{
    return mavlink_msg_set_velocity_body_pack(system_id, component_id, msg, set_velocity_body->time_boot_ms, set_velocity_body->target_system, set_velocity_body->target_component, set_velocity_body->vx, set_velocity_body->vy, set_velocity_body->vz);
}

/**
 * @brief Encode a set_velocity_body struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_velocity_body C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_velocity_body_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_velocity_body_t* set_velocity_body)
{
    return mavlink_msg_set_velocity_body_pack_chan(system_id, component_id, chan, msg, set_velocity_body->time_boot_ms, set_velocity_body->target_system, set_velocity_body->target_component, set_velocity_body->vx, set_velocity_body->vy, set_velocity_body->vz);
}

/**
 * @brief Send a set_velocity_body message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_velocity_body_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, vz);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY_BODY, buf, MAVLINK_MSG_ID_SET_VELOCITY_BODY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_CRC);
#else
    mavlink_set_velocity_body_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY_BODY, (const char *)&packet, MAVLINK_MSG_ID_SET_VELOCITY_BODY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_CRC);
#endif
}

/**
 * @brief Send a set_velocity_body message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_velocity_body_send_struct(mavlink_channel_t chan, const mavlink_set_velocity_body_t* set_velocity_body)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_velocity_body_send(chan, set_velocity_body->time_boot_ms, set_velocity_body->target_system, set_velocity_body->target_component, set_velocity_body->vx, set_velocity_body->vy, set_velocity_body->vz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY_BODY, (const char *)set_velocity_body, MAVLINK_MSG_ID_SET_VELOCITY_BODY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_velocity_body_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, vz);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY_BODY, buf, MAVLINK_MSG_ID_SET_VELOCITY_BODY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_CRC);
#else
    mavlink_set_velocity_body_t *packet = (mavlink_set_velocity_body_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY_BODY, (const char *)packet, MAVLINK_MSG_ID_SET_VELOCITY_BODY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_BODY_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_VELOCITY_BODY UNPACKING


/**
 * @brief Get field time_boot_ms from set_velocity_body message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_set_velocity_body_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from set_velocity_body message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_velocity_body_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from set_velocity_body message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_velocity_body_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field vx from set_velocity_body message
 *
 * @return X velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_velocity_body_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vy from set_velocity_body message
 *
 * @return Y velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_velocity_body_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vz from set_velocity_body message
 *
 * @return Z velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_velocity_body_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a set_velocity_body message into a struct
 *
 * @param msg The message to decode
 * @param set_velocity_body C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_velocity_body_decode(const mavlink_message_t* msg, mavlink_set_velocity_body_t* set_velocity_body)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_velocity_body->time_boot_ms = mavlink_msg_set_velocity_body_get_time_boot_ms(msg);
    set_velocity_body->vx = mavlink_msg_set_velocity_body_get_vx(msg);
    set_velocity_body->vy = mavlink_msg_set_velocity_body_get_vy(msg);
    set_velocity_body->vz = mavlink_msg_set_velocity_body_get_vz(msg);
    set_velocity_body->target_system = mavlink_msg_set_velocity_body_get_target_system(msg);
    set_velocity_body->target_component = mavlink_msg_set_velocity_body_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN? msg->len : MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN;
        memset(set_velocity_body, 0, MAVLINK_MSG_ID_SET_VELOCITY_BODY_LEN);
    memcpy(set_velocity_body, _MAV_PAYLOAD(msg), len);
#endif
}
