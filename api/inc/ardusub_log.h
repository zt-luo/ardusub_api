#pragma once

#include "ardusub_def.h"

#define MAX_LOG_STR (512)

static GAsyncQueue *log_str_queue;

void my_log_handler(const gchar *log_domain,
                    GLogLevelFlags log_level,
                    const gchar *message,
                    gpointer unused_data);

void as_set_log_handler();

gchar *pop_log_str();
void push_log_str(gchar *log_str);
