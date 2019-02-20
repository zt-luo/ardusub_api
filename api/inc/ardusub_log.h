/**
 * @file ardusub_log.h
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

#define MAX_LOG_STR (512)

void my_log_handler(const gchar *log_domain,
                    GLogLevelFlags log_level,
                    const gchar *message,
                    gpointer unused_data);

void as_set_log_handler();

gchar *pop_log_str();
void push_log_str(gchar *log_str);
