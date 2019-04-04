/**
 * @file ardusub_sqlite.h
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

#include "../../sqlite/sqlite3.h"

void as_sql_open_db();
void as_sql_close_db();
void as_sql_check_vechle_table(guint8 sys_id);
void as_sql_insert_vechle_table(guint8 sys_id, Vehicle_Data_t *vehicle_data);
void as_sql_check_test_info_table();
void as_sql_insert_test_info();
void as_sql_test_start(gchar *test_info, gchar *test_note);
void as_sql_test_stop();
