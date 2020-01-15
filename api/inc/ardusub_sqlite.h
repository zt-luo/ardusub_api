/**
 * @file ardusub_sqlite.h
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

typedef struct as_command_s
{
    uint8_t target_system;
    uint8_t depth_hold_cmd;
    float depth_hold_depth;
    uint8_t attitude_hold_cmd;
    float attitude_hold_yaw;
    float attitude_hold_pitch;
    float attitude_hold_roll;
    uint8_t flip_trick_type;
    float flip_trick_value;
} as_command_t;

#include <sqlite3.h>

void as_sql_open_db();
void as_sql_close_db();
void as_sql_check_vechle_table(guint8 sys_id);
void as_sql_insert_vechle_table(guint8 sys_id, Vehicle_Data_t *vehicle_data);
void as_sql_check_test_info_table();
void as_sql_insert_test_info();
void as_sql_test_start(const gchar *test_info, const gchar *test_note);
void as_sql_test_stop();
void as_sql_check_command_table();
void as_sql_insert_command(as_command_t as_command);
