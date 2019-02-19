#pragma once

#include "ardusub_def.h"

void my_log_handler(const gchar *log_domain,
                    GLogLevelFlags log_level,
                    const gchar *message,
                    gpointer unused_data);

void as_set_log_handler();
