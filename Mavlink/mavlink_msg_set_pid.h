#pragma once
// MESSAGE SET_PID PACKING

#define MAVLINK_MSG_ID_SET_PID 94

MAVPACKED(
typedef struct __mavlink_set_pid_t {
 float kP; /*< PID P gain*/
 float kI; /*< PID I gain*/
 float kD; /*< PID D gain*/
 float imax; /*< imax*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t controller_id; /*< controller ID*/
}) mavlink_set_pid_t;

#define MAVLINK_MSG_ID_SET_PID_LEN 19
#define MAVLINK_MSG_ID_SET_PID_MIN_LEN 19
#define MAVLINK_MSG_ID_94_LEN 19
#define MAVLINK_MSG_ID_94_MIN_LEN 19

#define MAVLINK_MSG_ID_SET_PID_CRC 79
#define MAVLINK_MSG_ID_94_CRC 79



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_PID { \
    94, \
    "SET_PID", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_pid_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_set_pid_t, target_component) }, \
         { "controller_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_set_pid_t, controller_id) }, \
         { "kP", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_pid_t, kP) }, \
         { "kI", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_pid_t, kI) }, \
         { "kD", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_pid_t, kD) }, \
         { "imax", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_pid_t, imax) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_PID { \
    "SET_PID", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_pid_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_set_pid_t, target_component) }, \
         { "controller_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_set_pid_t, controller_id) }, \
         { "kP", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_pid_t, kP) }, \
         { "kI", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_pid_t, kI) }, \
         { "kD", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_pid_t, kD) }, \
         { "imax", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_pid_t, imax) }, \
         } \
}
#endif

/**
 * @brief Pack a set_pid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param controller_id controller ID
 * @param kP PID P gain
 * @param kI PID I gain
 * @param kD PID D gain
 * @param imax imax
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_pid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t controller_id, float kP, float kI, float kD, float imax)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_PID_LEN];
    _mav_put_float(buf, 0, kP);
    _mav_put_float(buf, 4, kI);
    _mav_put_float(buf, 8, kD);
    _mav_put_float(buf, 12, imax);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, controller_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_PID_LEN);
#else
    mavlink_set_pid_t packet;
    packet.kP = kP;
    packet.kI = kI;
    packet.kD = kD;
    packet.imax = imax;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.controller_id = controller_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_PID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_PID;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_PID_MIN_LEN, MAVLINK_MSG_ID_SET_PID_LEN, MAVLINK_MSG_ID_SET_PID_CRC);
}

/**
 * @brief Pack a set_pid message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param controller_id controller ID
 * @param kP PID P gain
 * @param kI PID I gain
 * @param kD PID D gain
 * @param imax imax
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_pid_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t controller_id,float kP,float kI,float kD,float imax)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_PID_LEN];
    _mav_put_float(buf, 0, kP);
    _mav_put_float(buf, 4, kI);
    _mav_put_float(buf, 8, kD);
    _mav_put_float(buf, 12, imax);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, controller_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_PID_LEN);
#else
    mavlink_set_pid_t packet;
    packet.kP = kP;
    packet.kI = kI;
    packet.kD = kD;
    packet.imax = imax;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.controller_id = controller_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_PID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_PID;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_PID_MIN_LEN, MAVLINK_MSG_ID_SET_PID_LEN, MAVLINK_MSG_ID_SET_PID_CRC);
}

/**
 * @brief Encode a set_pid struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_pid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_pid_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_pid_t* set_pid)
{
    return mavlink_msg_set_pid_pack(system_id, component_id, msg, set_pid->target_system, set_pid->target_component, set_pid->controller_id, set_pid->kP, set_pid->kI, set_pid->kD, set_pid->imax);
}

/**
 * @brief Encode a set_pid struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_pid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_pid_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_pid_t* set_pid)
{
    return mavlink_msg_set_pid_pack_chan(system_id, component_id, chan, msg, set_pid->target_system, set_pid->target_component, set_pid->controller_id, set_pid->kP, set_pid->kI, set_pid->kD, set_pid->imax);
}

/**
 * @brief Send a set_pid message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param controller_id controller ID
 * @param kP PID P gain
 * @param kI PID I gain
 * @param kD PID D gain
 * @param imax imax
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_pid_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t controller_id, float kP, float kI, float kD, float imax)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_PID_LEN];
    _mav_put_float(buf, 0, kP);
    _mav_put_float(buf, 4, kI);
    _mav_put_float(buf, 8, kD);
    _mav_put_float(buf, 12, imax);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, controller_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID, buf, MAVLINK_MSG_ID_SET_PID_MIN_LEN, MAVLINK_MSG_ID_SET_PID_LEN, MAVLINK_MSG_ID_SET_PID_CRC);
#else
    mavlink_set_pid_t packet;
    packet.kP = kP;
    packet.kI = kI;
    packet.kD = kD;
    packet.imax = imax;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.controller_id = controller_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID, (const char *)&packet, MAVLINK_MSG_ID_SET_PID_MIN_LEN, MAVLINK_MSG_ID_SET_PID_LEN, MAVLINK_MSG_ID_SET_PID_CRC);
#endif
}

/**
 * @brief Send a set_pid message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_pid_send_struct(mavlink_channel_t chan, const mavlink_set_pid_t* set_pid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_pid_send(chan, set_pid->target_system, set_pid->target_component, set_pid->controller_id, set_pid->kP, set_pid->kI, set_pid->kD, set_pid->imax);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID, (const char *)set_pid, MAVLINK_MSG_ID_SET_PID_MIN_LEN, MAVLINK_MSG_ID_SET_PID_LEN, MAVLINK_MSG_ID_SET_PID_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_PID_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_pid_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t controller_id, float kP, float kI, float kD, float imax)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, kP);
    _mav_put_float(buf, 4, kI);
    _mav_put_float(buf, 8, kD);
    _mav_put_float(buf, 12, imax);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, controller_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID, buf, MAVLINK_MSG_ID_SET_PID_MIN_LEN, MAVLINK_MSG_ID_SET_PID_LEN, MAVLINK_MSG_ID_SET_PID_CRC);
#else
    mavlink_set_pid_t *packet = (mavlink_set_pid_t *)msgbuf;
    packet->kP = kP;
    packet->kI = kI;
    packet->kD = kD;
    packet->imax = imax;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->controller_id = controller_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID, (const char *)packet, MAVLINK_MSG_ID_SET_PID_MIN_LEN, MAVLINK_MSG_ID_SET_PID_LEN, MAVLINK_MSG_ID_SET_PID_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_PID UNPACKING


/**
 * @brief Get field target_system from set_pid message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_pid_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from set_pid message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_pid_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field controller_id from set_pid message
 *
 * @return controller ID
 */
static inline uint8_t mavlink_msg_set_pid_get_controller_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field kP from set_pid message
 *
 * @return PID P gain
 */
static inline float mavlink_msg_set_pid_get_kP(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field kI from set_pid message
 *
 * @return PID I gain
 */
static inline float mavlink_msg_set_pid_get_kI(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field kD from set_pid message
 *
 * @return PID D gain
 */
static inline float mavlink_msg_set_pid_get_kD(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field imax from set_pid message
 *
 * @return imax
 */
static inline float mavlink_msg_set_pid_get_imax(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a set_pid message into a struct
 *
 * @param msg The message to decode
 * @param set_pid C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_pid_decode(const mavlink_message_t* msg, mavlink_set_pid_t* set_pid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_pid->kP = mavlink_msg_set_pid_get_kP(msg);
    set_pid->kI = mavlink_msg_set_pid_get_kI(msg);
    set_pid->kD = mavlink_msg_set_pid_get_kD(msg);
    set_pid->imax = mavlink_msg_set_pid_get_imax(msg);
    set_pid->target_system = mavlink_msg_set_pid_get_target_system(msg);
    set_pid->target_component = mavlink_msg_set_pid_get_target_component(msg);
    set_pid->controller_id = mavlink_msg_set_pid_get_controller_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_PID_LEN? msg->len : MAVLINK_MSG_ID_SET_PID_LEN;
        memset(set_pid, 0, MAVLINK_MSG_ID_SET_PID_LEN);
    memcpy(set_pid, _MAV_PAYLOAD(msg), len);
#endif
}
