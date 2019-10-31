#pragma once
// MESSAGE NEW_LAND PACKING

#define MAVLINK_MSG_ID_NEW_LAND 96

MAVPACKED(
typedef struct __mavlink_new_land_t {
 float vx; /*< X velocity*/
 float vy; /*< Y velocity*/
 float vz; /*< Z velocity*/
 float disarm_height; /*< disarm_height*/
 float climb_rate; /*< climb_rate cms*/
 float throttle; /*< throttle 0-1*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
}) mavlink_new_land_t;

#define MAVLINK_MSG_ID_NEW_LAND_LEN 26
#define MAVLINK_MSG_ID_NEW_LAND_MIN_LEN 26
#define MAVLINK_MSG_ID_96_LEN 26
#define MAVLINK_MSG_ID_96_MIN_LEN 26

#define MAVLINK_MSG_ID_NEW_LAND_CRC 106
#define MAVLINK_MSG_ID_96_CRC 106



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NEW_LAND { \
    96, \
    "NEW_LAND", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_new_land_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_new_land_t, target_component) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_new_land_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_new_land_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_new_land_t, vz) }, \
         { "disarm_height", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_new_land_t, disarm_height) }, \
         { "climb_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_new_land_t, climb_rate) }, \
         { "throttle", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_new_land_t, throttle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NEW_LAND { \
    "NEW_LAND", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_new_land_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_new_land_t, target_component) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_new_land_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_new_land_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_new_land_t, vz) }, \
         { "disarm_height", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_new_land_t, disarm_height) }, \
         { "climb_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_new_land_t, climb_rate) }, \
         { "throttle", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_new_land_t, throttle) }, \
         } \
}
#endif

/**
 * @brief Pack a new_land message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param vx X velocity
 * @param vy Y velocity
 * @param vz Z velocity
 * @param disarm_height disarm_height
 * @param climb_rate climb_rate cms
 * @param throttle throttle 0-1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_new_land_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, float vx, float vy, float vz, float disarm_height, float climb_rate, float throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NEW_LAND_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, disarm_height);
    _mav_put_float(buf, 16, climb_rate);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NEW_LAND_LEN);
#else
    mavlink_new_land_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.disarm_height = disarm_height;
    packet.climb_rate = climb_rate;
    packet.throttle = throttle;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NEW_LAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NEW_LAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NEW_LAND_MIN_LEN, MAVLINK_MSG_ID_NEW_LAND_LEN, MAVLINK_MSG_ID_NEW_LAND_CRC);
}

/**
 * @brief Pack a new_land message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param vx X velocity
 * @param vy Y velocity
 * @param vz Z velocity
 * @param disarm_height disarm_height
 * @param climb_rate climb_rate cms
 * @param throttle throttle 0-1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_new_land_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,float vx,float vy,float vz,float disarm_height,float climb_rate,float throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NEW_LAND_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, disarm_height);
    _mav_put_float(buf, 16, climb_rate);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NEW_LAND_LEN);
#else
    mavlink_new_land_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.disarm_height = disarm_height;
    packet.climb_rate = climb_rate;
    packet.throttle = throttle;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NEW_LAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NEW_LAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NEW_LAND_MIN_LEN, MAVLINK_MSG_ID_NEW_LAND_LEN, MAVLINK_MSG_ID_NEW_LAND_CRC);
}

/**
 * @brief Encode a new_land struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param new_land C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_new_land_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_new_land_t* new_land)
{
    return mavlink_msg_new_land_pack(system_id, component_id, msg, new_land->target_system, new_land->target_component, new_land->vx, new_land->vy, new_land->vz, new_land->disarm_height, new_land->climb_rate, new_land->throttle);
}

/**
 * @brief Encode a new_land struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param new_land C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_new_land_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_new_land_t* new_land)
{
    return mavlink_msg_new_land_pack_chan(system_id, component_id, chan, msg, new_land->target_system, new_land->target_component, new_land->vx, new_land->vy, new_land->vz, new_land->disarm_height, new_land->climb_rate, new_land->throttle);
}

/**
 * @brief Send a new_land message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param vx X velocity
 * @param vy Y velocity
 * @param vz Z velocity
 * @param disarm_height disarm_height
 * @param climb_rate climb_rate cms
 * @param throttle throttle 0-1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_new_land_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float vx, float vy, float vz, float disarm_height, float climb_rate, float throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NEW_LAND_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, disarm_height);
    _mav_put_float(buf, 16, climb_rate);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_LAND, buf, MAVLINK_MSG_ID_NEW_LAND_MIN_LEN, MAVLINK_MSG_ID_NEW_LAND_LEN, MAVLINK_MSG_ID_NEW_LAND_CRC);
#else
    mavlink_new_land_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.disarm_height = disarm_height;
    packet.climb_rate = climb_rate;
    packet.throttle = throttle;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_LAND, (const char *)&packet, MAVLINK_MSG_ID_NEW_LAND_MIN_LEN, MAVLINK_MSG_ID_NEW_LAND_LEN, MAVLINK_MSG_ID_NEW_LAND_CRC);
#endif
}

/**
 * @brief Send a new_land message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_new_land_send_struct(mavlink_channel_t chan, const mavlink_new_land_t* new_land)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_new_land_send(chan, new_land->target_system, new_land->target_component, new_land->vx, new_land->vy, new_land->vz, new_land->disarm_height, new_land->climb_rate, new_land->throttle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_LAND, (const char *)new_land, MAVLINK_MSG_ID_NEW_LAND_MIN_LEN, MAVLINK_MSG_ID_NEW_LAND_LEN, MAVLINK_MSG_ID_NEW_LAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_NEW_LAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_new_land_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, float vx, float vy, float vz, float disarm_height, float climb_rate, float throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, disarm_height);
    _mav_put_float(buf, 16, climb_rate);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_LAND, buf, MAVLINK_MSG_ID_NEW_LAND_MIN_LEN, MAVLINK_MSG_ID_NEW_LAND_LEN, MAVLINK_MSG_ID_NEW_LAND_CRC);
#else
    mavlink_new_land_t *packet = (mavlink_new_land_t *)msgbuf;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->disarm_height = disarm_height;
    packet->climb_rate = climb_rate;
    packet->throttle = throttle;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_LAND, (const char *)packet, MAVLINK_MSG_ID_NEW_LAND_MIN_LEN, MAVLINK_MSG_ID_NEW_LAND_LEN, MAVLINK_MSG_ID_NEW_LAND_CRC);
#endif
}
#endif

#endif

// MESSAGE NEW_LAND UNPACKING


/**
 * @brief Get field target_system from new_land message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_new_land_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field target_component from new_land message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_new_land_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field vx from new_land message
 *
 * @return X velocity
 */
static inline float mavlink_msg_new_land_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy from new_land message
 *
 * @return Y velocity
 */
static inline float mavlink_msg_new_land_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vz from new_land message
 *
 * @return Z velocity
 */
static inline float mavlink_msg_new_land_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field disarm_height from new_land message
 *
 * @return disarm_height
 */
static inline float mavlink_msg_new_land_get_disarm_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field climb_rate from new_land message
 *
 * @return climb_rate cms
 */
static inline float mavlink_msg_new_land_get_climb_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field throttle from new_land message
 *
 * @return throttle 0-1
 */
static inline float mavlink_msg_new_land_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a new_land message into a struct
 *
 * @param msg The message to decode
 * @param new_land C-struct to decode the message contents into
 */
static inline void mavlink_msg_new_land_decode(const mavlink_message_t* msg, mavlink_new_land_t* new_land)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    new_land->vx = mavlink_msg_new_land_get_vx(msg);
    new_land->vy = mavlink_msg_new_land_get_vy(msg);
    new_land->vz = mavlink_msg_new_land_get_vz(msg);
    new_land->disarm_height = mavlink_msg_new_land_get_disarm_height(msg);
    new_land->climb_rate = mavlink_msg_new_land_get_climb_rate(msg);
    new_land->throttle = mavlink_msg_new_land_get_throttle(msg);
    new_land->target_system = mavlink_msg_new_land_get_target_system(msg);
    new_land->target_component = mavlink_msg_new_land_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NEW_LAND_LEN? msg->len : MAVLINK_MSG_ID_NEW_LAND_LEN;
        memset(new_land, 0, MAVLINK_MSG_ID_NEW_LAND_LEN);
    memcpy(new_land, _MAV_PAYLOAD(msg), len);
#endif
}
