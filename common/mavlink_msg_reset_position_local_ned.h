#pragma once
// MESSAGE RESET_POSITION_LOCAL_NED PACKING

#define MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED 50505


typedef struct __mavlink_reset_position_local_ned_t {
 int32_t latitude; /*< [degE7] Latitude (WGS84)*/
 int32_t longitude; /*< [degE7] Longitude (WGS84)*/
 int32_t altitude; /*< [mm] Altitude (MSL). Positive for up.*/
 uint16_t yaw; /*< [cdeg] */
 uint8_t target_system; /*<  System ID*/
} mavlink_reset_position_local_ned_t;

#define MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN 15
#define MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_MIN_LEN 15
#define MAVLINK_MSG_ID_50505_LEN 15
#define MAVLINK_MSG_ID_50505_MIN_LEN 15

#define MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_CRC 133
#define MAVLINK_MSG_ID_50505_CRC 133



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RESET_POSITION_LOCAL_NED { \
    50505, \
    "RESET_POSITION_LOCAL_NED", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_reset_position_local_ned_t, target_system) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_reset_position_local_ned_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_reset_position_local_ned_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_reset_position_local_ned_t, altitude) }, \
         { "yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_reset_position_local_ned_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RESET_POSITION_LOCAL_NED { \
    "RESET_POSITION_LOCAL_NED", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_reset_position_local_ned_t, target_system) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_reset_position_local_ned_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_reset_position_local_ned_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_reset_position_local_ned_t, altitude) }, \
         { "yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_reset_position_local_ned_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a reset_position_local_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param latitude [degE7] Latitude (WGS84)
 * @param longitude [degE7] Longitude (WGS84)
 * @param altitude [mm] Altitude (MSL). Positive for up.
 * @param yaw [cdeg] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reset_position_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, uint16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_uint16_t(buf, 12, yaw);
    _mav_put_uint8_t(buf, 14, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN);
#else
    mavlink_reset_position_local_ned_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.yaw = yaw;
    packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_CRC);
}

/**
 * @brief Pack a reset_position_local_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param latitude [degE7] Latitude (WGS84)
 * @param longitude [degE7] Longitude (WGS84)
 * @param altitude [mm] Altitude (MSL). Positive for up.
 * @param yaw [cdeg] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reset_position_local_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,int32_t latitude,int32_t longitude,int32_t altitude,uint16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_uint16_t(buf, 12, yaw);
    _mav_put_uint8_t(buf, 14, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN);
#else
    mavlink_reset_position_local_ned_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.yaw = yaw;
    packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_CRC);
}

/**
 * @brief Encode a reset_position_local_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param reset_position_local_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reset_position_local_ned_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_reset_position_local_ned_t* reset_position_local_ned)
{
    return mavlink_msg_reset_position_local_ned_pack(system_id, component_id, msg, reset_position_local_ned->target_system, reset_position_local_ned->latitude, reset_position_local_ned->longitude, reset_position_local_ned->altitude, reset_position_local_ned->yaw);
}

/**
 * @brief Encode a reset_position_local_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reset_position_local_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reset_position_local_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_reset_position_local_ned_t* reset_position_local_ned)
{
    return mavlink_msg_reset_position_local_ned_pack_chan(system_id, component_id, chan, msg, reset_position_local_ned->target_system, reset_position_local_ned->latitude, reset_position_local_ned->longitude, reset_position_local_ned->altitude, reset_position_local_ned->yaw);
}

/**
 * @brief Send a reset_position_local_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param latitude [degE7] Latitude (WGS84)
 * @param longitude [degE7] Longitude (WGS84)
 * @param altitude [mm] Altitude (MSL). Positive for up.
 * @param yaw [cdeg] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_reset_position_local_ned_send(mavlink_channel_t chan, uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, uint16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_uint16_t(buf, 12, yaw);
    _mav_put_uint8_t(buf, 14, target_system);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED, buf, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_CRC);
#else
    mavlink_reset_position_local_ned_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.yaw = yaw;
    packet.target_system = target_system;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED, (const char *)&packet, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_CRC);
#endif
}

/**
 * @brief Send a reset_position_local_ned message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_reset_position_local_ned_send_struct(mavlink_channel_t chan, const mavlink_reset_position_local_ned_t* reset_position_local_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_reset_position_local_ned_send(chan, reset_position_local_ned->target_system, reset_position_local_ned->latitude, reset_position_local_ned->longitude, reset_position_local_ned->altitude, reset_position_local_ned->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED, (const char *)reset_position_local_ned, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_CRC);
#endif
}

#if MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_reset_position_local_ned_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, uint16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_uint16_t(buf, 12, yaw);
    _mav_put_uint8_t(buf, 14, target_system);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED, buf, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_CRC);
#else
    mavlink_reset_position_local_ned_t *packet = (mavlink_reset_position_local_ned_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->yaw = yaw;
    packet->target_system = target_system;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED, (const char *)packet, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_CRC);
#endif
}
#endif

#endif

// MESSAGE RESET_POSITION_LOCAL_NED UNPACKING


/**
 * @brief Get field target_system from reset_position_local_ned message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_reset_position_local_ned_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field latitude from reset_position_local_ned message
 *
 * @return [degE7] Latitude (WGS84)
 */
static inline int32_t mavlink_msg_reset_position_local_ned_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from reset_position_local_ned message
 *
 * @return [degE7] Longitude (WGS84)
 */
static inline int32_t mavlink_msg_reset_position_local_ned_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from reset_position_local_ned message
 *
 * @return [mm] Altitude (MSL). Positive for up.
 */
static inline int32_t mavlink_msg_reset_position_local_ned_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field yaw from reset_position_local_ned message
 *
 * @return [cdeg] 
 */
static inline uint16_t mavlink_msg_reset_position_local_ned_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Decode a reset_position_local_ned message into a struct
 *
 * @param msg The message to decode
 * @param reset_position_local_ned C-struct to decode the message contents into
 */
static inline void mavlink_msg_reset_position_local_ned_decode(const mavlink_message_t* msg, mavlink_reset_position_local_ned_t* reset_position_local_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    reset_position_local_ned->latitude = mavlink_msg_reset_position_local_ned_get_latitude(msg);
    reset_position_local_ned->longitude = mavlink_msg_reset_position_local_ned_get_longitude(msg);
    reset_position_local_ned->altitude = mavlink_msg_reset_position_local_ned_get_altitude(msg);
    reset_position_local_ned->yaw = mavlink_msg_reset_position_local_ned_get_yaw(msg);
    reset_position_local_ned->target_system = mavlink_msg_reset_position_local_ned_get_target_system(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN? msg->len : MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN;
        memset(reset_position_local_ned, 0, MAVLINK_MSG_ID_RESET_POSITION_LOCAL_NED_LEN);
    memcpy(reset_position_local_ned, _MAV_PAYLOAD(msg), len);
#endif
}
