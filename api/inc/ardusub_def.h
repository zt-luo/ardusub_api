/**
 * @file ardusub_def.h
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-02-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include <glib.h>
#include <glib/gprintf.h>
#include <gio/gio.h>
#include <gmodule.h>

/******* MAVlink *******/
// max MAVlink channels
#define MAVLINK_COMM_NUM_BUFFERS (32)

#include <ardupilotmega/mavlink.h>
/******* MAVlink *******/


#include "ardusub_io.h"
#include "ardusub_api.h"
#include "ardusub_thread.h"
#include "ardusub_sqlite.h"
#include "ardusub_log.h"
#include "ardusub_ini.h"

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/* get more information about Parameter */
/* https://www.ardusub.com/operators-manual/full-parameter-list.html */
#define PARAM_COUNT (911)

#define MAX_ARDUSUB_TARGET_COUNT (253)

#define SUBNET_ADDRESS ("192.168.2.")

/* The maximum packet length is 279 bytes. */
/* 11 + 255(payload) + 13(signature) */
/* Please find more detials */
/* https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format */
#define MAX_BYTES (279)

#define STATION_SYSYEM_ID (255)
#define STATION_COMPONENT_ID (0)

#define MAX_STATUSTEX (512)
#define MAX_NAMED_VALUE_FLOAT (512)
#define MAX_MESSAGE (512)

#define MAX_SERIAL_PORT_WRITE_BUF_COUNT (512)

#define MIN_MSG_INTERVAL (1000) // in microseconds

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

typedef struct Time_Stamps_s
{
    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;
    uint64_t servo_output_raw;
    uint64_t command_ack;
    uint64_t named_value_float;
    uint64_t vfr_hud;
    uint64_t power_status;
    uint64_t system_time;
    uint64_t mission_current;
    uint64_t gps_raw_int;
    uint64_t nav_controller_output;
    uint64_t rc_channels;
    uint64_t vibration;
    uint64_t raw_imu;
    uint64_t scaled_pressure;
    uint64_t scaled_imu2;
    uint64_t scaled_pressure2;
    uint64_t rc_channels_raw;
    uint64_t statustext;
    uint64_t param_value;
    uint64_t timesync;
} Time_Stamps_t;

typedef struct messages_set_s
{

    int sysid;
    int compid;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;

    // System Status
    mavlink_sys_status_t sys_status;

    // Ping
    mavlink_ping_t ping;

    // Battery Status
    mavlink_battery_status_t battery_status;

    // Radio Status
    mavlink_radio_status_t radio_status;

    // Local Position
    mavlink_local_position_ned_t local_position_ned;

    // Global Position
    mavlink_global_position_int_t global_position_int;

    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;

    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;

    // HiRes IMU
    mavlink_highres_imu_t highres_imu;

    // Attitude
    mavlink_attitude_t attitude;

    // Servo Output Raw
    mavlink_servo_output_raw_t servo_output_raw;

    // Command ACK
    mavlink_command_ack_t command_ack;

    // Named Value Float
    mavlink_named_value_float_t named_value_float;

    // VFR_HUD
    mavlink_vfr_hud_t vfr_hud;

    // Power Status
    mavlink_power_status_t power_status;

    // System Time
    mavlink_system_time_t system_time;

    // Mission Current
    mavlink_mission_current_t mission_current;

    // GPS Raw INT
    mavlink_gps_raw_int_t gps_raw_int;

    // NAV Controller Output
    mavlink_nav_controller_output_t nav_controller_output;

    // RC Channels
    mavlink_rc_channels_t rc_channels;

    // Vibration
    mavlink_vibration_t vibration;

    // Raw IMU
    mavlink_raw_imu_t raw_imu;

    // Scaled Pressure
    mavlink_scaled_pressure_t scaled_pressure;

    // Scaled IMU2
    mavlink_scaled_imu2_t scaled_imu2;

    // Scaled Pressure (depth sensor)
    mavlink_scaled_pressure2_t scaled_pressure2;

    // RC channels raw
    mavlink_rc_channels_raw_t rc_channels_raw;

    // StatusText
    mavlink_statustext_t statustext;

    // Param Value
    mavlink_param_value_t param_value;

    // Timesync
    mavlink_timesync_t timesync;

    // System Parameters? not here.

    // Time Stamps
    Time_Stamps_t time_stamps;

    guint msg_id;

} Mavlink_Messages_t;

typedef struct Mavlink_Parameter_s
{
    char param_id[16];
    mavlink_param_union_t param_value;
    enum MAV_PARAM_TYPE param_type; // NOTE: if param_type == 0 , then this Parameter is empty.
} Mavlink_Parameter_t;
