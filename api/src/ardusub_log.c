#define G_LOG_DOMAIN "[ardusub log       ]"

#include "../inc/ardusub_log.h"

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

    GError *error = NULL;
    gsize bytes_written;
    GIOChannel *api_log_file_ch = g_io_channel_new_file("ardusub_api_log.txt", "a", &error);
    g_io_channel_write_chars(api_log_file_ch, log_str, -1, &bytes_written, &error);
    g_io_channel_flush(api_log_file_ch, &error);
    g_io_channel_unref(api_log_file_ch);

    g_date_time_unref(data_time);
    g_free(data_time_str);
    g_free(log_str);
}

void as_set_log_handler()
{
    g_log_set_default_handler(my_log_handler, NULL);
}
