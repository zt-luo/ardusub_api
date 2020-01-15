/**
 * @file ardusub_io.h
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

#ifndef NO_SERISL
#include <libserialport.h>
#endif

char *subnet_address;

GHashTable *target_hash_table;

GAsyncQueue *serial_write_buf_queue[MAVLINK_COMM_NUM_BUFFERS];

GRWLock target_hash_table_lock;

extern guint8 *sys_key[255];

void as_udp_read_init();
void as_udp_write_init(guint8 sysid, GSocket *p_target_socket);

#ifndef NO_SERISL
void as_serial_read_init();
void as_serial_write_init();
#endif

gboolean as_find_new_system(mavlink_message_t message,
                            guint8 *targer_serial_chan);

void send_mavlink_message(guint8 target_system, mavlink_message_t *message);

void send_heartbeat(guint8 target_system);
void send_param_request_list(guint8 target_system, guint8 target_autopilot);
void send_param_request_read(guint8 target_system, guint8 target_component, gint16 param_index);

#ifndef NO_SERISL
gchar *serial_write_buf_queue_pop(guint8 chan);
void serial_write_buf_queue_push(guint8 chan, gchar *buf, gsize buf_len);
#endif
