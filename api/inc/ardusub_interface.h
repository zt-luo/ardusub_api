/**
 * @file ardusub_interface.h
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-02-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include "ardusub_def.h"

// ------------------------------------------------------------------------------
//   Variables
// ------------------------------------------------------------------------------

volatile guint sys_count;
volatile gboolean as_init_status;
volatile unsigned int thread_flag;

// atomic operations should apply on this
guint8 *sys_key[255];
system_status_t vehicle_status[255];
control_mode_t vehicle_mode[255];

// maybe a hash table is no need, a simple array is enough
GHashTable *message_hash_table;
GHashTable *parameter_hash_table;
GHashTable *manual_control_table;

Vehicle_Data_t *vehicle_data_array[255];

// globle mutex
GMutex message_mutex[255];
GMutex parameter_mutex[255];
GMutex manual_control_mutex[255];
GMutex vehicle_data_mutex[255];

GRWLock message_hash_table_lock;
GRWLock parameter_hash_table_lock;
GRWLock manual_control_hash_table_lock;

GAsyncQueue *statustex_queue[255];
GAsyncQueue *named_val_float_queue[255];
GAsyncQueue *message_queue[255];

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

//
// public api

void as_api_init(const char *subnet_address, const unsigned int flag);
void as_api_deinit();
int as_api_check_vehicle(uint8_t sysid);
void as_api_vehicle_arm(uint8_t target_system, uint8_t target_autopilot);
void as_api_vehicle_disarm(uint8_t target_system, uint8_t target_autopilot);
void as_api_manual_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, ...);
int as_api_statustex_count(uint8_t target_system);
mavlink_statustext_t *as_api_statustex_queue_pop(uint8_t target_system);
mavlink_named_value_float_t *as_api_named_val_float_queue_pop(guint8 target_system);
Vehicle_Data_t *as_api_get_vehicle_data(uint8_t target_system);
int as_api_get_vehicle_data2(uint8_t target_system, Vehicle_Data_t *vehicle_data);
void as_api_set_mode(uint8_t target_system, control_mode_t mode);
void as_api_set_servo(uint8_t target_system, uint8_t target_autopilot,
                      float servo_no, float pwm);
void as_api_motor_test(uint8_t target_system, uint8_t target_autopilot,
                       float motor_no, float pwm);
void as_api_send_rc_channels_override(uint8_t target_system, uint8_t target_autopilot,
                                      uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4,
                                      uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8);
void as_api_send_named_value_float(uint8_t target_system, char *name, float value);
void as_api_send_named_value_int(uint8_t target_system, char *name, int value);
void as_api_test_start(const char *test_info, const char *test_note);
void as_api_test_stop();
void as_api_depth_hold(uint8_t target_system, uint8_t cmd, float depth);
void as_api_attitude_hold(uint8_t target_system, uint8_t cmd, float yaw, float pitch, float roll);
void as_api_flip_trick(uint8_t target_system, uint8_t type, float value);
void as_api_depth_pid(uint8_t target_system, uint8_t save, float kp,
                      float ki, float kd, float imax, float filt_hz, float ff);

//
// func inside high leval

void *as_run(gpointer data);
void as_system_add(guint8 target_system, guint8 target_autopilot,
                   Mavlink_Messages_t *current_messages,
                   Mavlink_Parameter_t *current_parameter,
                   GSocket *current_target_socket,
                   guint8 *current_targer_serial_port);

Mavlink_Messages_t *as_get_message(uint8_t sysid);

void as_request_full_parameters(guint8 target_system, guint8 target_component);

void as_send_request_data_stream(guint8 target_system, guint8 target_component,
                                 guint8 req_stream_id, guint16 req_message_rate,
                                 guint8 start_stop);
void as_reauest_data_stream(guint8 target_system, guint8 target_component);

void as_insert_command(as_command_t as_commadn);

//
// func inside low leval

mavlink_statustext_t *statustex_queue_pop(guint8 target_system);
void statustex_queue_push(guint8 target_system, Mavlink_Messages_t *current_messages);

mavlink_named_value_float_t *named_val_float_queue_pop(guint8 target_system);
void named_val_float_queue_push(guint8 target_system, Mavlink_Messages_t *current_messages);

Mavlink_Messages_t *message_queue_pop(guint8 target_system);
void message_queue_push(guint8 target_system, Mavlink_Messages_t *current_messages);
