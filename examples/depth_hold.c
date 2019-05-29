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
    as_api_init(NULL, F_THREAD_FETCH_FULL_PARAM |
                          F_THREAD_NAMED_VAL_FLOAT |
                          F_THREAD_STATUSTEX_WALL);

    g_message("Checking if system 1 is active...");

    while (0 == as_api_check_vehicle(1))
    {
        ;
    }

    g_message("system 1 is active.");

    g_message("set LAB_REMOTE mode.");
    as_api_set_mode(1, LAB_ALT_HOLD);
    // as_api_set_mode(1, LAB_REMOTE);
    g_usleep(1000000);

    as_api_test_start("depth_hold", "");
    g_message("vehicle arm");
    as_api_vehicle_arm(1, 1);

    as_api_depth_hold(1, 1, -0.5);
    // as_api_attitude_hold(1, 1, 2.0, 3.0, 4.0);
    // as_api_flip_trick(1, 1, 6.0);
    // as_api_depth_pid(1, 0, 1.1, 1.2, 1.3, 0, 0, 0);

    g_usleep(1000000);

    as_api_depth_hold(1, 1, -0.2);
    g_usleep(1000000);
    g_usleep(1000000);
    g_usleep(1000000);

    getchar();

    as_api_vehicle_disarm(1, 1);
    as_api_test_stop();
    as_api_deinit();

    return 0;
}
