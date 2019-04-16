/**
 * @file depth_hold.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-04-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#define G_LOG_DOMAIN "[depth_hold        ]"

#include <stdio.h>

#include <glib.h>

#include "../api/inc/ardusub_api.h"

int main()
{
    as_api_init(NULL);

    g_message("Checking if system 1 is active...");

    while (0 == as_api_check_vehicle(1))
    {
        ;
    }

    g_message("system 1 is active.");

    as_api_depth_hold(1, 1, 2.0);
    as_api_attitude_hold(1, 1, 2.0, 3.0, 4.0);
    as_api_flip_trick(1, 1, 6.0);

    g_usleep(1000000);

    g_print("\nstatustex count:%d\n",
            as_api_statustex_count(1));
    mavlink_statustext_t *statustxt = as_api_statustex_queue_pop(1);
    for (gint i = 0; i < 50 && statustxt != NULL; i++)
    {
        g_print("severity:%d, text:%s\n",
                statustxt->severity,
                statustxt->text);
        statustxt = as_api_statustex_queue_pop(1);
    }

    getchar();

    as_api_deinit();

    g_usleep(1000000);

    return 0;
}
