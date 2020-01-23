/**
 * @file ardusub_thread.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-02-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#define G_LOG_DOMAIN "[ardusub thread    ]"

#include "../inc/ardusub_msg.h"
#include "../inc/ardusub_thread.h"
#include "../inc/ardusub_interface.h"
#include "../inc/ardusub_sqlite.h"

#ifndef NO_SERISL
gint serial_port_thread_count = 0;
#endif

gint db_insert_command_thread_count = 0;

/**
 * @brief init thread prt and running flag
 * 
 */
void as_thread_init_ptr_flag()
{
    //
    // thread ptr
    for (gsize i = 0; i < 255; i++)
    {
        manual_control_thread[i] = NULL;
    }

    for (gsize i = 0; i < 255; i++)
    {
        named_val_float_handle_thread[i] = NULL;
    }

    for (gsize i = 0; i < 255; i++)
    {
        vehicle_data_update_thread[i] = NULL;
    }

    for (gsize i = 0; i < 255; i++)
    {
        db_update_thread[i] = NULL;
    }

    for (gsize i = 0; i < 255; i++)
    {
        statustex_wall_thread[i] = NULL;
    }

    parameters_request_thread = NULL;
    request_data_stream_thread = NULL;
    log_str_write_thread = NULL;

    //
    // thread running flag

    for (gsize i = 0; i < 255; i++)
    {
        manual_control_worker_run[i] = 1;
    }

    for (gsize i = 0; i < 255; i++)
    {
        named_val_float_handle_worker_run[i] = 1;
    }

    for (gsize i = 0; i < 255; i++)
    {
        vehicle_data_update_worker_run[i] = 1;
    }

    for (gsize i = 0; i < 255; i++)
    {
        db_update_worker_run[i] = 1;
    }

    for (gsize i = 0; i < 255; i++)
    {
        statustex_wall_worker_run[i] = 1;
    }

    for (gsize i = 0; i < 255; i++)
    {
        heartbeat_worker_run[i] = 1;
    }

    log_str_write_worker_run = 1;
}

/**
 * @brief stop all thread and join 
 * 
 */
void as_thread_stop_all_join()
{
    // exit main loop
    if (as_main_loop != NULL && TRUE == g_main_loop_is_running(as_main_loop))
    {
        g_main_loop_quit(as_main_loop);
    }
    g_thread_join(as_api_main_thread);
    g_message("exit main loop.");

#ifndef NO_SERISL
    if (NULL == subnet_address) // this means serial port connection
    {
        // wait serial port thread exit
        while (0 != g_atomic_int_get(&serial_port_thread_count))
        {
            as_thread_msleep(10);
        }
        g_message("exit all serial port thread.");
    }
#endif

    for (gsize i = 0; i < 255; i++)
    {
        if (SYS_UN_INIT != g_atomic_int_get(vehicle_status + i))
        {
            // send stop signal
            g_atomic_int_set(manual_control_worker_run + i, 0);
            g_atomic_int_set(named_val_float_handle_worker_run + i, 0);
            g_atomic_int_set(vehicle_data_update_worker_run + i, 0);
            g_atomic_int_set(db_update_worker_run + i, 0);
            g_atomic_int_set(statustex_wall_worker_run + i, 0);
            g_atomic_int_set(heartbeat_worker_run + i, 0);

            // join
            GThread *this_thread;
            this_thread = manual_control_thread[i];
            if (NULL != this_thread)
            {
                g_thread_join(this_thread);
            }
            manual_control_thread[i] = NULL;
            
            this_thread = named_val_float_handle_thread[i];
            if (NULL != this_thread)
            {
                g_thread_join(this_thread);
            }
            named_val_float_handle_thread[i] = NULL;

            this_thread = vehicle_data_update_thread[i];
            if (NULL != this_thread)
            {
                g_thread_join(this_thread);
            }
            vehicle_data_update_thread[i] = NULL;

            this_thread = db_update_thread[i];
            if (NULL != this_thread)
            {
                g_thread_join(this_thread);
            }
            db_update_thread[i] = NULL;

            this_thread = statustex_wall_thread[i];
            if (NULL != this_thread)
            {
                g_thread_join(this_thread);
            }
            statustex_wall_thread[i] = NULL;

            this_thread = heartbeat_thread[i];
            if (NULL != this_thread)
            {
                g_thread_join(this_thread);
            }
            heartbeat_thread[i] = NULL;
        }
    }

    // send stop signal
    g_atomic_int_set(&log_str_write_worker_run, 0);

    // join
    if (NULL != log_str_write_thread)
    {
        g_thread_join(log_str_write_thread);
    }

    if (NULL != parameters_request_thread)
    {
        g_thread_join(parameters_request_thread);
    }
    if (NULL != request_data_stream_thread)
    {
        g_thread_join(request_data_stream_thread);
    }

    // wait db_insert_command thread exit
    while (0 != g_atomic_int_get(&db_insert_command_thread_count))
    {
        as_thread_msleep(10);
    }
    g_message("exit all db_insert_command thread.");

    // close database
    as_sql_close_db();
}

/**
 * @brief manual_control_worker
 * 
 * @param data 
 * @return gpointer 
 */
gpointer manual_control_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;
    gpointer system_key_ = g_atomic_pointer_get(sys_key + my_target_system);
    g_assert(NULL != system_key_);

    while (1 == g_atomic_int_get(manual_control_worker_run + my_target_system))
    {
        if (SYS_ARMED == g_atomic_int_get(vehicle_status + my_target_system) &&
            MANUAL == g_atomic_int_get(vehicle_mode + my_target_system)) // Atomic Operation
        {
            g_rw_lock_reader_lock(&manual_control_hash_table_lock);
            mavlink_manual_control_t *my_manual_control =
                g_hash_table_lookup(manual_control_table, system_key_);
            g_rw_lock_reader_unlock(&manual_control_hash_table_lock);

            g_mutex_lock(&manual_control_mutex[my_target_system]); // lock
            mavlink_manual_control_t *safe_manual_control =
                g_memdup(my_manual_control, sizeof(mavlink_manual_control_t)); // memdup
            g_mutex_unlock(&manual_control_mutex[my_target_system]);           // unlock

            mavlink_message_t message;
            mavlink_msg_manual_control_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID,
                                              &message, safe_manual_control);
            send_mavlink_message(my_target_system, &message);

            g_free(safe_manual_control); // free

            // g_usleep(100000);
            as_thread_msleep(100);
        }
        else
        {
            // g_usleep(100000);
            as_thread_msleep(20);
        }
    }

    g_message("exit manual_control_worker, sysid: %d.", my_target_system);

    return NULL;
}

/**
 * @brief parameters_request_worker
 * 
 * @param data 
 * @return gpointer 
 */
gpointer parameters_request_worker(gpointer data)
{
    guint16 target_ = 0;
    target_ = *(guint16 *)data;
    g_free(data);
    guint8 target_system = 0, target_component = 0xFF;
    target_system = target_ >> 8;
    target_component &= target_;

    send_param_request_list(target_system, target_component); // no guarantee
    as_thread_msleep(3000);

    g_rw_lock_reader_lock(&parameter_hash_table_lock);
    Mavlink_Parameter_t *current_parameter =
        g_hash_table_lookup(parameter_hash_table,
                            g_atomic_pointer_get(sys_key + target_system));
    g_rw_lock_reader_unlock(&parameter_hash_table_lock);

    for (gsize j = 0; j < 10; j++)
    {
        if (FALSE == g_main_loop_is_running(as_main_loop))
        {
            break;
        }

        for (gsize i = 0; i < PARAM_COUNT; i++)
        {
            if (FALSE == g_main_loop_is_running(as_main_loop))
            {
                break;
            }

            g_mutex_lock(&parameter_mutex[target_system]);
            gchar param_id_stx = current_parameter[i].param_id[0];
            g_mutex_unlock(&parameter_mutex[target_system]);

            if (0 == param_id_stx)
            {
                send_param_request_read(target_system, target_component, i);
                as_thread_msleep(100);
            }
        }

        as_thread_msleep(100);
    }

    g_mutex_lock(&parameter_mutex[target_system]);
    for (gsize i = 0; i < PARAM_COUNT; i++)
    {
        if (FALSE == g_main_loop_is_running(as_main_loop))
        {
            break;
        }

        if (0 == current_parameter[i].param_id[0])
        {
            if (FALSE == g_main_loop_is_running(as_main_loop))
            {
                break;
            }

            g_message("Fetch all parameter FAILED!");
            return NULL;
        }
    }
    g_mutex_unlock(&parameter_mutex[target_system]);

    g_message("Fetch all parameter SUCCEED!");

    return NULL;
}

/**
 * @brief request_data_stream_worker
 * 
 * @param data 
 * @return gpointer 
 */
gpointer request_data_stream_worker(gpointer data)
{
    guint16 target_ = 0;
    target_ = *(guint16 *)data;
    g_free(data);
    guint8 target_system = 0, target_component = 0xFF;
    target_system = target_ >> 8;
    target_component &= target_;

    // g_message("start request_data_stream.");
    // g_message("target_system: %d, target_component: %d.", target_system, target_component);

    as_send_request_data_stream(target_system, target_component,
                                1, 10, 1);
    as_thread_msleep(500);
    as_send_request_data_stream(target_system, target_component,
                                2, 4, 1);
    as_thread_msleep(500);
    as_send_request_data_stream(target_system, target_component,
                                3, 4, 1);
    as_thread_msleep(500);
    as_send_request_data_stream(target_system, target_component,
                                6, 10, 1);
    as_thread_msleep(500);
    as_send_request_data_stream(target_system, target_component,
                                10, 10, 1);
    as_thread_msleep(500);
    as_send_request_data_stream(target_system, target_component,
                                11, 10, 1);
    as_thread_msleep(500);
    as_send_request_data_stream(target_system, target_component,
                                12, 3, 1);
    as_thread_msleep(500);

    // g_message("finish request_data_stream.");

    return NULL;
}

/**
 * @brief named_val_float_handle_worker
 * 
 * @param data 
 * @return gpointer 
 */
gpointer named_val_float_handle_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;

    // wait for named_val_float_queue ready
    while (NULL == g_atomic_pointer_get(
                       named_val_float_queue + my_target_system))
    {
        as_thread_msleep(100);
    }

    mavlink_named_value_float_t *my_named_value_float =
        named_val_float_queue_pop(my_target_system);

    while (1 == g_atomic_int_get(named_val_float_handle_worker_run + my_target_system))
    {
        if (NULL != my_named_value_float)
        {
            //TODO: save this values to somewhere.
            // g_message("%s: %f",
            //           my_named_value_float->name,
            //           my_named_value_float->value);
        }
        else
        {
            as_thread_msleep(10);
        }

        my_named_value_float =
            named_val_float_queue_pop(my_target_system);
    }

    g_message("exit named_val_float_handle_worker, sysid: %d.", my_target_system);

    return NULL;
}

/**
 * @brief vehicle_data_update_worker
 * 
 * @param data 
 * @return gpointer 
 */
gpointer vehicle_data_update_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;

    while (NULL == g_atomic_pointer_get(vehicle_data_array + my_target_system))
    {
        as_thread_msleep(10);
    }

    Vehicle_Data_t *my_vehicle_data = g_atomic_pointer_get(vehicle_data_array + my_target_system);
    g_assert(NULL != my_vehicle_data);

    Mavlink_Messages_t *my_mavlink_message = message_queue_pop(my_target_system);

    while (1 == g_atomic_int_get(vehicle_data_update_worker_run + my_target_system))
    {
        if (NULL != my_mavlink_message)
        {
            mavlink_heartbeat_t hb = my_mavlink_message->heartbeat;
            mavlink_sys_status_t ss = my_mavlink_message->sys_status;
            mavlink_battery_status_t bs = my_mavlink_message->battery_status;
            mavlink_power_status_t ps = my_mavlink_message->power_status;
            mavlink_system_time_t st = my_mavlink_message->system_time;
            mavlink_attitude_t at = my_mavlink_message->attitude;
            mavlink_scaled_pressure_t sp = my_mavlink_message->scaled_pressure;
            mavlink_scaled_pressure2_t sp2 = my_mavlink_message->scaled_pressure2;
            mavlink_servo_output_raw_t sor = my_mavlink_message->servo_output_raw;
            mavlink_raw_imu_t ri = my_mavlink_message->raw_imu;
            mavlink_rc_channels_t rc = my_mavlink_message->rc_channels;
            mavlink_global_position_int_t gpi = my_mavlink_message->global_position_int;

            g_mutex_lock(&vehicle_data_mutex[my_target_system]);
            // update vehicle data here

            switch (my_mavlink_message->msg_id)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
                my_vehicle_data->custom_mode = hb.custom_mode;
                my_vehicle_data->type = hb.type;
                my_vehicle_data->autopilot = hb.autopilot;
                my_vehicle_data->base_mode = hb.system_status;
                my_vehicle_data->system_status = hb.system_status;
                my_vehicle_data->mavlink_version = hb.mavlink_version;
                break;

            case MAVLINK_MSG_ID_SYS_STATUS:
                my_vehicle_data->onboard_control_sensors_present =
                    ss.onboard_control_sensors_present;
                my_vehicle_data->onboard_control_sensors_enabled =
                    ss.onboard_control_sensors_enabled;
                my_vehicle_data->onboard_control_sensors_health =
                    ss.onboard_control_sensors_health;
                my_vehicle_data->load = ss.load;
                my_vehicle_data->voltage_battery = ss.voltage_battery;
                my_vehicle_data->current_battery = ss.current_battery;
                my_vehicle_data->drop_rate_comm = ss.drop_rate_comm;
                my_vehicle_data->errors_comm = ss.errors_comm;
                my_vehicle_data->errors_count1 = ss.errors_count1;
                my_vehicle_data->errors_count2 = ss.errors_count2;
                my_vehicle_data->errors_count3 = ss.errors_count3;
                my_vehicle_data->errors_count4 = ss.errors_count4;
                my_vehicle_data->battery_remaining = ss.battery_remaining;
                break;

            case MAVLINK_MSG_ID_BATTERY_STATUS:
                my_vehicle_data->current_consumed = bs.current_consumed;
                my_vehicle_data->energy_consumed = bs.energy_consumed;
                my_vehicle_data->temperature_bs = bs.temperature;
                memcpy(my_vehicle_data->voltages, bs.voltages, sizeof(uint16_t) * 10);
                my_vehicle_data->current_battery_bs = bs.current_battery;
                my_vehicle_data->battery_id = bs.id; //! multiple battery?
                my_vehicle_data->battery_function = bs.battery_function;
                my_vehicle_data->type_bs = bs.type;
                my_vehicle_data->battery_remaining_bs = bs.battery_remaining;
                my_vehicle_data->time_remaining = bs.time_remaining;
                my_vehicle_data->charge_state = bs.charge_state;
                break;

            case MAVLINK_MSG_ID_POWER_STATUS:
                my_vehicle_data->Vcc_ps = ps.Vcc;
                my_vehicle_data->Vservo_ps = ps.Vservo;
                my_vehicle_data->flags_ps = ps.flags;
                break;

            case MAVLINK_MSG_ID_SYSTEM_TIME:
                my_vehicle_data->time_unix_usec = st.time_unix_usec;
                my_vehicle_data->time_boot_ms = st.time_boot_ms;
                break;

            case MAVLINK_MSG_ID_ATTITUDE:
                my_vehicle_data->time_boot_ms_at = at.time_boot_ms;
                my_vehicle_data->roll = at.roll;
                my_vehicle_data->pitch = at.pitch;
                my_vehicle_data->yaw = at.yaw;
                my_vehicle_data->rollspeed = at.rollspeed;
                my_vehicle_data->pitchspeed = at.pitchspeed;
                my_vehicle_data->yawspeed = at.yawspeed;
                break;

            case MAVLINK_MSG_ID_SCALED_PRESSURE:
                my_vehicle_data->time_boot_ms_sp = sp.time_boot_ms;
                my_vehicle_data->press_abs = sp.press_abs;
                my_vehicle_data->press_diff = sp.press_diff;
                break;

            case MAVLINK_MSG_ID_SCALED_PRESSURE2:
                my_vehicle_data->time_boot_ms_sp2 = sp2.time_boot_ms;
                my_vehicle_data->press_abs2 = sp2.press_abs;
                my_vehicle_data->press_diff2 = sp2.press_diff;
                break;

            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                my_vehicle_data->time_usec_sor = sor.time_usec;
                my_vehicle_data->servo1_raw = sor.servo1_raw;
                my_vehicle_data->servo2_raw = sor.servo2_raw;
                my_vehicle_data->servo3_raw = sor.servo3_raw;
                my_vehicle_data->servo4_raw = sor.servo4_raw;
                my_vehicle_data->servo5_raw = sor.servo5_raw;
                my_vehicle_data->servo6_raw = sor.servo6_raw;
                my_vehicle_data->servo7_raw = sor.servo7_raw;
                my_vehicle_data->servo8_raw = sor.servo8_raw;
                my_vehicle_data->port = sor.port;
                my_vehicle_data->servo9_raw = sor.servo9_raw;
                my_vehicle_data->servo10_raw = sor.servo10_raw;
                my_vehicle_data->servo11_raw = sor.servo11_raw;
                my_vehicle_data->servo12_raw = sor.servo12_raw;
                my_vehicle_data->servo13_raw = sor.servo13_raw;
                my_vehicle_data->servo14_raw = sor.servo14_raw;
                my_vehicle_data->servo15_raw = sor.servo15_raw;
                my_vehicle_data->servo16_raw = sor.servo16_raw;
                break;

            case MAVLINK_MSG_ID_RAW_IMU:
                my_vehicle_data->time_usec_ri = ri.time_usec;
                my_vehicle_data->xacc = ri.xacc;
                my_vehicle_data->yacc = ri.yacc;
                my_vehicle_data->zacc = ri.zacc;
                my_vehicle_data->xgyro = ri.xgyro;
                my_vehicle_data->ygyro = ri.ygyro;
                my_vehicle_data->zgyro = ri.zgyro;
                my_vehicle_data->xmag = ri.xmag;
                my_vehicle_data->ymag = ri.ymag;
                my_vehicle_data->zmag = ri.zmag;
                break;

            case MAVLINK_MSG_ID_RC_CHANNELS:
                my_vehicle_data->time_boot_ms_rc = rc.time_boot_ms;
                my_vehicle_data->chan1_raw = rc.chan1_raw;
                my_vehicle_data->chan2_raw = rc.chan2_raw;
                my_vehicle_data->chan3_raw = rc.chan3_raw;
                my_vehicle_data->chan4_raw = rc.chan4_raw;
                my_vehicle_data->chan5_raw = rc.chan5_raw;
                my_vehicle_data->chan6_raw = rc.chan6_raw;
                my_vehicle_data->chan7_raw = rc.chan7_raw;
                my_vehicle_data->chan8_raw = rc.chan8_raw;
                my_vehicle_data->chan9_raw = rc.chan9_raw;
                my_vehicle_data->chan10_raw = rc.chan10_raw;
                my_vehicle_data->chan11_raw = rc.chan11_raw;
                my_vehicle_data->chan12_raw = rc.chan12_raw;
                my_vehicle_data->chan13_raw = rc.chan13_raw;
                my_vehicle_data->chan14_raw = rc.chan14_raw;
                my_vehicle_data->chan15_raw = rc.chan15_raw;
                my_vehicle_data->chan16_raw = rc.chan16_raw;
                my_vehicle_data->chan17_raw = rc.chan17_raw;
                my_vehicle_data->chan18_raw = rc.chan18_raw;
                my_vehicle_data->chancount = rc.chancount;
                my_vehicle_data->rssi = rc.rssi;
                break;

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                my_vehicle_data->time_boot_ms_gpi = gpi.time_boot_ms;
                my_vehicle_data->lat = gpi.lat;
                my_vehicle_data->lon = gpi.lon;
                my_vehicle_data->alt = gpi.alt;
                my_vehicle_data->relative_alt = gpi.relative_alt;
                my_vehicle_data->vx = gpi.vx;
                my_vehicle_data->vy = gpi.vy;
                my_vehicle_data->vz = gpi.vz;
                my_vehicle_data->hdg = gpi.hdg;
                
                break;

            default:
                break;
            }

            g_mutex_unlock(&vehicle_data_mutex[my_target_system]);
        }
        else
        {
            as_thread_msleep(10);
        }
        my_mavlink_message = message_queue_pop(my_target_system);
    }

    g_message("exit vehicle_data_update_worker, sysid: %d.", my_target_system);

    return NULL;
}

/**
 * @brief db_update_worker
 * 
 * @param data 
 * @return gpointer 
 */
gpointer db_update_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;

    as_sql_check_vechle_table(my_target_system);

    Vehicle_Data_t *my_vehicle_data = g_atomic_pointer_get(vehicle_data_array + my_target_system);
    g_assert(NULL != my_vehicle_data);

    while (1 == g_atomic_int_get(db_update_worker_run + my_target_system))
    {
        as_sql_insert_vechle_table(my_target_system, my_vehicle_data);

        as_thread_msleep(10);
    }

    // stop test
    as_sql_test_stop();

    g_message("exit db_update_worker, sysid: %d.", my_target_system);

    return NULL;
}

/**
 * @brief log_str_write_worker
 * 
 * @param data 
 * @return gpointer 
 */
gpointer log_str_write_worker(gpointer data)
{
    g_assert(NULL == data);

    GError *error = NULL;
    gsize bytes_written;
    GIOChannel *api_log_file_ch = g_io_channel_new_file("ardusub_api_log.txt", "a", &error);

    gchar *log_str = pop_log_str();

    while (1 == g_atomic_int_get(&log_str_write_worker_run))
    {
        if (NULL != log_str)
        {
            g_io_channel_write_chars(api_log_file_ch, log_str, -1, &bytes_written, &error);
            g_io_channel_flush(api_log_file_ch, &error);
        }
        else
        {
            as_thread_msleep(10);
        }

        log_str = pop_log_str();
    }

    g_io_channel_unref(api_log_file_ch);

    g_message("exit log_str_write_worker.");

    return NULL;
}

/**
 * @brief udp_read_callback
 * 
 * @param channel 
 * @param condition 
 * @param data 
 * @return gboolean 
 */
gboolean udp_read_callback(GIOChannel *channel,
                           GIOCondition condition,
                           gpointer data)
{
    g_assert(NULL == data);

    // g_print("Received data from client!\n");

    gchar msg_tmp[MAX_BYTES] = {'\0'};
    gsize bytes_read;
    GError *error = NULL;

    if (condition & G_IO_HUP)
    {
        return FALSE; /* this channel is done */
    }

    g_io_channel_read_chars(channel, msg_tmp, sizeof(msg_tmp), &bytes_read, &error);

    /* don't forget to check for errors */
    if (error != NULL)
    {
        g_error(error->message);
    }

    // g_print("%s", msg_tmp);

    mavlink_message_t message;
    mavlink_status_t status;
    gboolean msg_received = FALSE;

    for (gsize i = 0; i < (bytes_read < MAX_BYTES ? bytes_read : MAX_BYTES) && FALSE == msg_received; i++)
    {
        msg_received =
            mavlink_parse_char(MAVLINK_COMM_1, msg_tmp[i], &message, &status);
    }

    if (TRUE == msg_received)
    {
        as_find_new_system(message, NULL);

        as_handle_messages(message);
    }

    return TRUE;
}

/**
 * @brief serial_port_read_write_worker
 * 
 * @param data 
 * @return gpointer 
 */
#ifndef NO_SERISL
gpointer serial_port_read_write_worker(gpointer data)
{
    struct sp_port *my_serial_port = NULL;
    my_serial_port = (struct sp_port *)data;

    static guint8 serial_chan;

    guint8 my_chan = g_atomic_int_get((gint *)&serial_chan);

    if (my_chan == MAVLINK_COMM_NUM_BUFFERS - 1)
    {
        g_error("MAVLINK_COMM_NUM_BUFFERS reached!");
    }

    g_atomic_int_inc(&serial_port_thread_count);

    g_atomic_int_inc((gint *)&serial_chan);

    guint8 buf;
    gboolean msg_received = FALSE;
    gboolean fin_new_system = FALSE;
    mavlink_message_t message;
    mavlink_status_t status;

    enum sp_return sp_rt = SP_OK;

    while (NULL == as_main_loop)
    {
        as_thread_msleep(10);
        g_message("wait main loop");
    }

    while (g_main_loop_is_running(as_main_loop))
    {
        gchar *write_buf = serial_write_buf_queue_pop(my_chan);
        if (NULL != write_buf)
        {
            sp_rt = sp_blocking_write(my_serial_port, write_buf + 1, write_buf[0], 0);
            if (SP_OK > sp_rt)
            {
                g_error("failed in serial port write: %d", sp_rt);
            }
        }

        sp_rt = sp_blocking_read(my_serial_port, &buf, 1, 0);
        if (SP_OK > sp_rt)
        {
            g_error("failed in serial port read: %d", sp_rt);
        }

        msg_received =
            mavlink_parse_char(my_chan, buf, &message, &status);

        if (TRUE == msg_received)
        {
            if (FALSE == fin_new_system)
            {
                // find new system
                fin_new_system = as_find_new_system(message, &my_chan);
            }

            as_handle_messages(message);
        }
    }

    g_atomic_int_dec_and_test(&serial_port_thread_count);

    return NULL;
}
#endif

gpointer db_insert_command_worker(gpointer data)
{
    g_assert(NULL != data);
    as_command_t *as_command = (as_command_t *)data;

    g_atomic_int_inc(&db_insert_command_thread_count);

    as_sql_check_command_table();

    as_sql_insert_command(*as_command);

    g_free(as_command);

    // g_message("exit db_insert_command_worker");
    g_atomic_int_dec_and_test(&db_insert_command_thread_count);

    return NULL;
}

gpointer statustex_wall_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;

    mavlink_statustext_t *statustxt;
    gchar *severity_tex[8] = {"EMERGENCY", "ALERT",
                              "CRITICAL", "ERROR",
                              "WARNING", "NOTICE",
                              "INFO", "DEBUG"};
    GDateTime *data_time;
    gchar *data_time_str;

    while (1 == g_atomic_int_get(statustex_wall_worker_run + my_target_system))
    {
        statustxt = as_api_statustex_queue_pop(my_target_system);

        if (NULL != statustxt)
        {
            data_time = g_date_time_new_now_local();
            data_time_str = g_date_time_format(data_time, "%F %T");
            g_print("[%s:%06d] %s : %s\n",
                    data_time_str,
                    g_date_time_get_microsecond(data_time),
                    severity_tex[statustxt->severity],
                    statustxt->text);

            g_date_time_unref(data_time);
            g_free(data_time_str);
        }
        else
        {
            as_thread_msleep(10);
        }
    }

    g_message("exit statustex_wall_worker.");

    return NULL;
}

gpointer heartbeat_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;

    while (1 == g_atomic_int_get(heartbeat_worker_run + my_target_system))
    {
        send_heartbeat(my_target_system);

        as_thread_msleep(500);
    }

    g_message("exit heartbeat_worker.");

    return NULL;
}

void as_thread_msleep(gint ms)
{
    for (gint i = 0; i < ms; i++)
    {
        if (as_main_loop != NULL && TRUE == g_main_loop_is_running(as_main_loop))
        {
            g_usleep(1000);
        }
        else
        {
            break;
        }
    }
}
