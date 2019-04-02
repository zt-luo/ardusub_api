/**
 * @file ardusub_ini.h
 * @author Zongtong Luo (luozongtong123@163.com)
 * @brief 
 * @version 
 * @date 2019-04-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include "ardusub_def.h"

gboolean as_config_log_file;
gboolean as_config_log_stdout;

void as_read_ini_file();
void as_init_ini_file();

