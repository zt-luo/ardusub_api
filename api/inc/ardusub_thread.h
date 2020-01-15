/**
 * @file ardusub_thread.h
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

// TODO: command watch thread
// check command ack msg

//
// thread ptr
GMainLoop *as_main_loop;
GThread *manual_control_thread[255];
GThread *parameters_request_thread;
GThread *request_data_stream_thread;
GThread *named_val_float_handle_thread[255];
GThread *vehicle_data_update_thread[255];
GThread *db_update_thread[255];
GThread *statustex_wall_thread[255];
GThread *heartbeat_thread[255];
GThread *log_str_write_thread;
GThread *as_api_main_thread;

//
// thread running flag
volatile gint manual_control_worker_run[255];
volatile gint named_val_float_handle_worker_run[255];
volatile gint vehicle_data_update_worker_run[255];
volatile gint db_update_worker_run[255];
volatile gint statustex_wall_worker_run[255];
volatile gint heartbeat_worker_run[255];
volatile gint log_str_write_worker_run;

void as_thread_init_ptr_flag();
void as_thread_stop_all_join();
void as_thread_msleep(gint ms);

//
// thread worker func
gpointer manual_control_worker(gpointer data);
gpointer parameters_request_worker(gpointer data);
gpointer request_data_stream_worker(gpointer data);
gpointer named_val_float_handle_worker(gpointer data);
gpointer vehicle_data_update_worker(gpointer data);
gpointer db_update_worker(gpointer data);
gpointer log_str_write_worker(gpointer data);
gpointer db_insert_command_worker(gpointer data);
gpointer statustex_wall_worker(gpointer data);
gpointer heartbeat_worker(gpointer data);
gboolean udp_read_callback(GIOChannel *channel,
                           GIOCondition condition,
                           gpointer socket_udp_write); // udp read worker

#ifndef NO_SERISL
gpointer serial_port_read_write_worker(gpointer data);
#endif
