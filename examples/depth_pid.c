/**
 * @file depth_pid.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-04-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#define G_LOG_DOMAIN "[depth_pid         ]"
#define D_PER_RAD (180.0 / 3.1415926)

#include <stdio.h>

#include <glib.h>

#include "../api/inc/ardusub_api.h"

volatile int depth_controller_run = 1;
float yaw, pitch, roll, depth;

void update_vehicle_data();
gpointer depth_controller(gpointer data);
GThread *depth_controller_thread;

/**
 * @brief 
 * 
 * @return int 
 */
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

    g_message("start depth_controller");
    depth_controller_thread = g_thread_new("depth_controller",
                                           &depth_controller, NULL);

    getchar();
    g_atomic_int_set(&depth_controller_run, 0);
    g_thread_join(depth_controller_thread);

    as_api_deinit();

    return 0;
}

void update_vehicle_data()
{
    Vehicle_Data_t *vehicle_data;

    vehicle_data = as_api_get_vehicle_data(1);

    yaw = vehicle_data->yaw * D_PER_RAD;
    pitch = vehicle_data->pitch * D_PER_RAD;
    roll = vehicle_data->roll * D_PER_RAD;
    depth = vehicle_data->alt / 1000.0; // m
    // depth = vehicle_data->press_abs2 / 9800.0; // m

    g_message("yaw: %f, pitch: %f, roll: %f, depth: %f m.\n", yaw, pitch, roll, depth);
}

gpointer depth_controller(gpointer data)
{
    g_assert(NULL == data);

    as_api_test_start("depth_control", NULL);

    g_message("set LAB_REMOTE mode.");
    as_api_set_mode(1, LAB_REMOTE);

    g_message("vehicle arm");
    as_api_vehicle_arm(1, 1);

    double z_d = -0.5; // [m] desired depth
    int pwm_out = 1500;

    gint64 tnow, dt = 50000;

    // gains for depth controller
    double K_P = 300;
    double K_I = 0;
    double K_D = 0;

    int pwm_limit = 150;

    double I_term_max = -600; // I_term_min and max prevent integral windup by saturating the I_term
    double I_term_min = -700; //
    double z_vel_lim = 1;     // [m/s] this is added to prevent setpoint kick in the derivative term

    double P_term = 0;
    double I_term = 0;
    double D_term = 0;
    double z_now = 0; // depth in this iteration
    double z_old = 0; // depth in the previous iteration
    double z_err = 0; // declare the depth error
    double z_vel = 0; // declare the derivative term for z

    while (1 == g_atomic_int_get(&depth_controller_run))
    {
        tnow = g_get_monotonic_time();
        update_vehicle_data();
        // g_message("update time : %ld.", g_get_monotonic_time() - tnow);

        // update ROV's state
        z_old = z_now;
        z_now = depth;
        z_err = z_d - z_now;
        z_vel = (z_now - z_old) / (dt / 1000);

        // clamp the value of z_vel
        if (z_vel < -z_vel_lim)
        {
            z_vel = -z_vel_lim;
        }
        if (z_vel > z_vel_lim)
        {
            z_vel = z_vel_lim;
        }

        P_term = K_P * z_err;
        if (K_I == 0)
        {
            I_term = 0;
        }
        else
        {
            if (z_d < -0.2) // dont do this while disarm
            {
                I_term = I_term + K_I * z_err * (dt / 1000);
                // clamp the value of I_term
                if (I_term < I_term_min)
                {
                    I_term = I_term_min;
                }
                if (I_term > I_term_max)
                {
                    I_term = I_term_max;
                }
            }
        }
        D_term = K_D * z_vel;

        pwm_out = P_term + I_term + D_term;

        // clamp the value of pwm_out
        if (pwm_out < -pwm_limit)
        {
            pwm_out = -pwm_limit;
        }
        if (pwm_out > pwm_limit)
        {
            pwm_out = pwm_limit;
        }

        as_api_send_rc_channels_override(1, 1,
                                         1500, 1500,
                                         1500, 1500,
                                         1500 + pwm_out,
                                         1500 - pwm_out,
                                         1500 - pwm_out,
                                         1500 + pwm_out);

        // as_api_send_rc_channels_override(1, 1,
        //                                  1500, 1500,
        //                                  1500, 1500,
        //                                  1500 - pwm_out,
        //                                  1500 + pwm_out,
        //                                  1500 - pwm_out,
        //                                  1500 + pwm_out);

        while (tnow + dt > g_get_monotonic_time())
        {
            g_usleep(20);
        }

        // g_print("time error: %ld.\n", tnow + dt - g_get_monotonic_time());
        g_print("pwm_out: %d\n", pwm_out);
    }

    g_message("vehicle disarm");
    as_api_vehicle_disarm(1, 1);

    g_message("set MANUAL mode.");
    as_api_set_mode(1, MANUAL);

    return NULL;
}
