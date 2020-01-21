/**
 * @file simple_example.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 0.1
 * @date 2018-10-16
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#define G_LOG_DOMAIN "[simple_example    ]"

#include <stdio.h>

#include <glib.h>

#include "../api/inc/ardusub_api.h"

gpointer monitoring_thread(gpointer data);

/**
 * @brief 
 * 
 * @param parameter_size 
 * @param parameter 
 * @return int 
 */
int main(int argc, char *argv[])
{
    char s = 0;

    if (argc > 1)
    {
        s = argv[1][0];
    }
    else
    {
        g_print("input 1 or 2 or 3: ");
        scanf("%c", &s);
    }

    if ('1' == s)
    {
        g_message("subnet address 192.168.%c.*", s);
        as_api_init("192.168.1.", F_THREAD_FETCH_FULL_PARAM |
                                      F_THREAD_NAMED_VAL_FLOAT |
                                      F_THREAD_STATUSTEX_WALL);
    }
    else if ('2' == s)
    {
        g_message("subnet address 192.168.%c.*", s);
        as_api_init(NULL, F_THREAD_FETCH_FULL_PARAM |
                              F_THREAD_NAMED_VAL_FLOAT |
                              F_THREAD_STATUSTEX_WALL);
    }
    else if ('3' == s)
    {
        g_message("connecting to serial port.");
        as_api_init("serial port", F_THREAD_FETCH_FULL_PARAM |
                                       F_THREAD_NAMED_VAL_FLOAT |
                                       F_THREAD_STATUSTEX_WALL);
    }
    else
    {
        g_error("invalde input!");
    }

    g_message("Checking if system 1 is active...");

    while (0 == as_api_check_vehicle(1))
    {
        ;
    }

    g_message("system 1 is active.");

    g_message("Start monitoring...");
    g_thread_new("monitoring_thread",
                 &monitoring_thread, NULL);

    g_message("vehicle arm...");
    as_api_vehicle_arm(1, 1);

    g_usleep(2000000);

    as_api_manual_control(100, 0, 500, 0, 0);

    g_usleep(2000000);
    g_usleep(2000000);
    g_usleep(2000000);

    mavlink_statustext_t *statustxt;

    g_print("\nstatustex count:%d\n",
            as_api_statustex_count(1));
    statustxt = as_api_statustex_queue_pop(1);
    for (gint i = 0; i < 50 && statustxt != NULL; i++)
    {
        g_print("severity:%d, text:%s\n",
                statustxt->severity,
                statustxt->text);
        statustxt = as_api_statustex_queue_pop(1);
    }
    g_print("\n");

    g_usleep(2000000);

    g_message("vehicle disarm...");
    as_api_vehicle_disarm(1, 1);

    g_usleep(2000000);
    g_message("vehicle arm...");
    as_api_vehicle_arm(1, 1);
    as_api_manual_control(100, 0, 500, 0, 0);
    g_usleep(2000000);
    g_usleep(2000000);
    g_usleep(2000000);
    g_usleep(2000000);
    g_usleep(2000000);
    g_usleep(2000000);
    g_message("vehicle disarm...");
    as_api_vehicle_disarm(1, 1);

    as_api_deinit();

    return 0;
}

/**
 * @brief get and print vehicle data.
 * 
 * @param data 
 * @return gpointer 
 */
gpointer monitoring_thread(gpointer data)
{
    g_assert(NULL == data);
    Vehicle_Data_t *vehicle_data;

    while (TRUE)
    {
        vehicle_data = as_api_get_vehicle_data(1);

        if (NULL == vehicle_data)
        {
            continue;
        }

        g_print("\n===================================\n");
        g_print("System ID: %d\n", 1);
        g_print("System status: %d\n", vehicle_data->system_status);
        g_print("Time unix: %llu\n", vehicle_data->time_unix_usec);
        g_print("Mode: %d\n", vehicle_data->base_mode);
        g_print("MAVLink version: %d\n", vehicle_data->mavlink_version);
        g_print("------------------------------------\n");
        g_print("Battery voltage: %d\n", vehicle_data->voltage_battery);
        g_print("Battery current: %d\n", vehicle_data->current_battery);
        g_print("------------------------------------\n");
        g_print("Roll: %f\n", vehicle_data->roll);
        g_print("Pitch: %f\n", vehicle_data->pitch);
        g_print("Yaw: %f\n", vehicle_data->yaw);
        g_print("Roll speed: %f\n", vehicle_data->rollspeed);
        g_print("Pitch speed: %f\n", vehicle_data->pitchspeed);
        g_print("Yaw speed: %f\n", vehicle_data->yawspeed);
        g_print("------------------------------------\n");
        g_print("Depth sensor:\n %f(abs), %f(diff)\n", vehicle_data->press_abs2, vehicle_data->press_diff2);
        g_print("------------------------------------\n");
        g_print("Servo output raw:\n");
        g_print("servo1_raw: %d\n", vehicle_data->servo1_raw);
        g_print("servo2_raw: %d\n", vehicle_data->servo2_raw);
        g_print("servo3_raw: %d\n", vehicle_data->servo3_raw);
        g_print("servo4_raw: %d\n", vehicle_data->servo4_raw);
        g_print("servo5_raw: %d\n", vehicle_data->servo5_raw);
        g_print("servo6_raw: %d\n", vehicle_data->servo6_raw);
        g_print("servo7_raw: %d\n", vehicle_data->servo7_raw);
        g_print("servo8_raw: %d\n", vehicle_data->servo8_raw);
        g_print("====================================\n");

        g_usleep(2000000);
    }

    return NULL;
}
