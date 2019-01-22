#pragma once

#include "ardusub_def.h"

char *subnet_address;

guint8 udp_write_ready[255];

GHashTable *target_socket_hash_table;

GRWLock target_socket_hash_table_lock;

extern guint8 *sys_key[255];

void as_udp_read_init();
void as_udp_write_init(guint8 sysid, GSocket *p_target_socket);

void as_serial_read_init(char *serial);
void as_serial_write_init(guint8 sysid);

void send_udp_message(guint8 target_system, mavlink_message_t *message);

void send_heartbeat(guint8 target_system);
void send_param_request_list(guint8 target_system, guint8 target_autopilot);
void send_param_request_read(guint8 target_system, guint8 target_component, gint16 param_index);

// callbcak func
gboolean udp_read_callback(GIOChannel *channel,
                           GIOCondition condition,
                           gpointer socket_udp_write);
