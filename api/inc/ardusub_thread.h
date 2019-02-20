/**
 * @file ardusub_thread.h
 * @author Zongtong Luo (luozongtong123@163.com)
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
// thread worker func
gpointer manual_control_worker(gpointer data);
gpointer parameters_request_worker(gpointer data);
gpointer request_data_stream_worker(gpointer data);
gpointer named_val_float_handle_worker(gpointer data);
gpointer vehicle_data_update_worker(gpointer data);
gpointer db_update_worker(gpointer data);
gpointer log_str_write_worker(gpointer data);
