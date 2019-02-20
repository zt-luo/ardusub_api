#define G_LOG_DOMAIN "[ardusub log       ]"

#include "../inc/ardusub_log.h"

static GAsyncQueue *log_str_queue;

void my_log_handler(const gchar *log_domain,
                    GLogLevelFlags log_level,
                    const gchar *message,
                    gpointer unused_data)
{

    g_assert(NULL == unused_data);

    gchar *log_level_str;
    GDateTime *data_time = g_date_time_new_now_local();
    gchar *data_time_str = g_date_time_format(data_time, "%F %T");

    switch (log_level)
    {

    case G_LOG_LEVEL_ERROR:
        log_level_str = "[Error   ";
        break;

    case G_LOG_LEVEL_CRITICAL:
        log_level_str = "[Critical]";
        break;

    case G_LOG_LEVEL_WARNING:
        log_level_str = "[Warning ]";
        break;

    case G_LOG_LEVEL_MESSAGE:
        log_level_str = "[Message ]";
        break;

    case G_LOG_LEVEL_INFO:
        log_level_str = "[Info    ]";
        break;

    case G_LOG_LEVEL_DEBUG:
        log_level_str = "[Debug   ]";
        break;

    default:
        log_level_str = "[Error   ]";
        break;
    }

    gchar *log_str = g_new0(gchar, 512);
    g_snprintf(log_str,
               512,
               "** %s %s %s:%06d -> %s\n",
               log_domain,
               log_level_str,
               data_time_str,
               g_date_time_get_microsecond(data_time),
               message);

    push_log_str(log_str);

    g_date_time_unref(data_time);
    g_free(data_time_str);
    g_free(log_str);
}

void as_set_log_handler()
{
    log_str_queue = g_async_queue_new();

    g_log_set_default_handler(my_log_handler, NULL);

    g_thread_new("log_str_write_worker", &log_str_write_worker, NULL);
}

gchar *pop_log_str()
{
    static gchar *last_log_str;
    static GMutex my_mutex;

    g_mutex_lock(&my_mutex);
    // only one thread can reach here

    if (NULL != last_log_str)
    {
        // free last log_str after pop
        g_free(last_log_str);
    }

    last_log_str = g_async_queue_try_pop(log_str_queue);

    g_mutex_unlock(&my_mutex);

    return last_log_str;
}

void push_log_str(gchar *log_str)
{
    g_assert(NULL != log_str);

    if (g_async_queue_length(log_str_queue) > MAX_LOG_STR)
    {
        g_critical("MAX_LOG_STR reached!");
    }

    gchar *log_str_p = g_strdup_printf(log_str);

    if (NULL == log_str_p)
    {
        g_error("Out of memory!");
    }

    g_async_queue_push(log_str_queue, log_str_p);
}
