/**
 * @file simple_example.c
 * @author Zongtong Luo (luozongtong123@163.com)
 * @brief 
 * @version 0.1
 * @date 2018-10-16
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <stdio.h>

#include <glib.h>

#include "../lib/inc/ardusub_api.h"

/**
 * @brief 
 * 
 * @param parameter_size 
 * @param parameter 
 * @return int 
 */
int main(int argc, char *argv[])
{
    as_api_init("192.168.1.");

    // as_api_run();

    g_print("Creating as_api_run_worker thread...\n");
    GThread *as_api_run_worker = g_thread_new("as_api_run_worker", (GThreadFunc)as_api_run, NULL);
    g_usleep(2000000);

    g_print("Checking if system 1 is active...\n");

    while (0 == as_api_check_active_sys(1))
    {
        ;
    }

    g_print("system 1 is active.\n");

    g_print("vehicle arm...\n");
    vehicle_arm();

    g_usleep(2000000);

    as_api_manual_control(200, 0, 0, 0, 0);

    g_usleep(2000000);
    g_usleep(2000000);
    g_usleep(2000000);
    g_usleep(2000000);

    g_print("vehicle disarm...\n");
    vehicle_disarm();
    g_usleep(2000000);

    while (1)
        ;
    return 0;
}
