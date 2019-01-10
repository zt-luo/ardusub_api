#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include <glib.h>
#include <glib/gprintf.h>
#include <gio/gio.h>
#include <gmodule.h>

#include <common/mavlink.h>

#include "ardusub_api.h"

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/* get more information about Parameter */
/* https://www.ardusub.com/operators-manual/full-parameter-list.html */
#define PARAM_COUNT (689)

#define MAX_ARDUSUB_TARGET_COUNT (253)

/* The maximum packet length is 279 bytes. */
/* 11 + 255(payload) + 13(signature) */
/* Please find more detials */
/* https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format */
#define MAX_BYTES (279)

#define STATION_SYSYEM_ID (255)
#define STATION_COMPONENT_ID (0)

#define SUBNET_ADDRESS ("192.168.2.")

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

    // System Parameters?

    // Time Stamps
    Time_Stamps_t time_stamps;

} Mavlink_Messages_t;

typedef struct Mavlink_Parameter_s
{
    char param_id[16];
    mavlink_param_union_t param_value;
    enum MAV_PARAM_TYPE param_type; // NOTE: if param_type == 0 , then this Parameter is empty.
} Mavlink_Parameter_t;

#ifdef __cplusplus
extern "C"
{
#endif

    // ------------------------------------------------------------------------------
    //   Variables
    // ------------------------------------------------------------------------------

    static char heartbeat_writing_status;
    static char control_status;

    //current target system id
    static int current_target_system;
    static int current_target_autopilot;

    static guint system_count;

    static gboolean as_init_status;

    static char* subnet_address;

    /* START only manipulated by as_handle_messages() START */
    static Mavlink_Messages_t *current_messages;
    static Mavlink_Parameter_t *current_parameter;
    static GSocket *current_target_socket;
    static guint8 *system_key[255];
    static volatile guint8 arm_status[255];
    /* STOP  only manipulated by as_handle_messages()  STOP */

    GHashTable *message_hash_table;
    GHashTable *parameter_hash_table;
    GHashTable *target_socket_hash_table;
    GHashTable *manual_control_table;

    //globle mutex
    GMutex message_mutex;
    GMutex parameter_mutex;
    GMutex target_socket_mutex;
    GMutex manual_control_mutex[255];

    // ------------------------------------------------------------------------------
    //   Prototypes
    // ------------------------------------------------------------------------------
    void as_api_init(char* subnet_address);
    void as_api_deinit();
    void as_api_run();
    void as_sys_add(uint8_t sysid);

    int as_api_check_active_sys(uint8_t sysid);

    void as_api_manual_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, ...);

        Mavlink_Messages_t *as_get_meaasge(uint8_t sysid);

    void as_udp_write_init(guint8 sysid, GSocket *p_target_socket);
    void as_udp_read_init();

    guint8 as_handle_messages(gchar *msg_tmp, gsize bytes_read);
    void as_handle_message_id(mavlink_message_t message);
    int as_write_message(mavlink_message_t message);
    void manual_control_worker(gpointer data);

    void enable_offboard_control();
    void disable_offboard_control();
    void vehicle_arm(guint8 target_system);
    void vehicle_disarm(guint8 target_system);

    void do_set_servo(float servo_no, float pwm);
    void do_motor_test(float motor_no, float pwm);
    void do_set_mode(control_mode_t mode_);
    void request_param_list(void);
    void send_rc_channels_override(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4,
                                   uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8);
    void send_heartbeat(void);

    // TODO:
    // Serial_Port *serial_port;

    int toggle_offboard_control(bool flag);

    gboolean udp_read_callback(GIOChannel *channel,
                               GIOCondition condition,
                               gpointer socket_udp_write);

    void send_udp_message(mavlink_message_t *message);
    void sendto_udp_message(GSocket *target_socket, mavlink_message_t *message);

    // ------------------------------------------------------------------------------
    //   Inline Functions
    // ------------------------------------------------------------------------------
