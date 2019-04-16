/**
 * @file ardusub_ini.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-04-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#define G_LOG_DOMAIN "[ardusub ini       ]"

#include "../inc/ardusub_ini.h"

/**
 * @brief read config file. if not exist, creat one.
 * 
 */
void as_read_ini_file()
{
    g_autoptr(GError) error = NULL;
    g_autoptr(GKeyFile) key_file = g_key_file_new();

    as_config_log_file = TRUE;
    as_config_log_stdout = TRUE;

    // load ini file
    GKeyFileFlags flag = G_KEY_FILE_KEEP_COMMENTS;
    if (!g_key_file_load_from_file(key_file, "ardusub_config.ini", flag, &error))
    {
        if (G_FILE_ERROR_NOENT == error->code)
        {
            g_warning("NO ardusub_config.ini file. %s", error->message);
            g_clear_error(&error);
            as_init_ini_file();
        }
        else
        {
            g_error("Error init config file: %s", error->message);

            return;
        }
    }
    else
    {
        as_config_log_file = g_key_file_get_boolean(key_file, "log", "file", &error);
        as_config_log_stdout = g_key_file_get_boolean(key_file, "log", "stdout", &error);
    }
}

/**
 * @brief creat config file, write default config
 * 
 */
void as_init_ini_file()
{
    g_autoptr(GKeyFile) key_file = g_key_file_new();
    g_autoptr(GError) error = NULL;

    gchar ini_top_comment[] = "This is ini file for ardusub api.";
    g_key_file_set_comment(key_file, NULL, NULL, ini_top_comment, &error);
    g_clear_error(&error);

    g_key_file_set_boolean(key_file, "log", "file", TRUE);
    g_clear_error(&error);

    g_key_file_set_boolean(key_file, "log", "stdout", TRUE);
    g_clear_error(&error);

    g_key_file_set_comment(key_file, "log", NULL, "log config", &error);
    g_clear_error(&error);

    g_key_file_set_comment(key_file, "log", "file", "log to file?", &error);
    g_clear_error(&error);

    g_key_file_set_comment(key_file, "log", "stdout", "log to stdout?", &error);
    g_clear_error(&error);

    // Save as a file.
    g_info("creating config file.");
    if (!g_key_file_save_to_file(key_file, "ardusub_config.ini", &error))
    {
        g_warning("Error saving ardusub_config.ini: %s", error->message);

        return;
    }
}

