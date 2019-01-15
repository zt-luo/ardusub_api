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
    char s = 0;

    if (argc > 1)
    {
        s = argv[1][0];
    }
    else
    {
        g_print("input 1 or 2 : ");
        scanf("%c", &s);
    }

    g_print("subnet address 192.168.%c.*\n", s);

    if ('1' == s)
    {
        as_api_init("192.168.1.");
    }
    else if ('2' == s)
    {
        as_api_init(NULL);
    }
    else
    {
        g_error("invalde input!");
    }

    // as_api_run();

    g_print("Checking if system 1 is active...\n");

    while (0 == as_api_check_vehicle(1))
    {
        ;
    }

    g_print("system 1 is active.\n");

    g_print("vehicle arm...\n");
    as_api_vehicle_arm(1, 1);

    g_usleep(2000000);

    as_api_manual_control(100, 0, 500, 0, 0);

    g_usleep(2000000);
    g_usleep(2000000);
    g_usleep(2000000);

    mavlink_statustext_t *statustxt;

    g_print("statustex count:%d\n", as_api_statustex_cpunt(1));
    statustxt = as_api_statustex_queue_pop(1);
    for (gint i = 0; i < 50 && statustxt != NULL; i++)
    {
        g_print("severity:%d, text:%s\n", statustxt->severity, statustxt->text);
        statustxt = as_api_statustex_queue_pop(1);
    }

    g_usleep(2000000);

    g_print("vehicle disarm...\n");
    as_api_vehicle_disarm(1, 1);

    g_usleep(2000000);
    as_api_vehicle_arm(1, 1);
    as_api_manual_control(100, 0, 500, 0, 0);
    g_usleep(2000000);
    g_usleep(2000000);
    as_api_vehicle_disarm(1, 1);

    while (1)
        ;
    scanf("%s", &s);

    return 0;
}
