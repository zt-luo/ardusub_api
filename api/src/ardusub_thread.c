#include "../inc/ardusub_interface.h"

gpointer manual_control_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;
    gpointer system_key_ = g_atomic_pointer_get(sys_key + my_target_system);
    g_assert(NULL != system_key_);

    while (TRUE)
    {
        if (SYS_ARMED == g_atomic_int_get(vehicle_status + my_target_system)) // Atomic Operation
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

            g_usleep(100000);
        }
        else
        {
            g_usleep(100);
        }
    }

    return NULL;
}

gpointer parameters_request_worker(gpointer data)
{
    guint16 target_ = 0;
    target_ = *(guint16 *)data;
    g_free(data);
    guint8 target_system = 0, target_component = 0xFF;
    target_system = target_ >> 8;
    target_component &= target_;

    send_param_request_list(target_system, target_component); // no guarantee
    g_usleep(3000000);

    g_rw_lock_reader_lock(&parameter_hash_table_lock);
    Mavlink_Parameter_t *current_parameter =
        g_hash_table_lookup(parameter_hash_table,
                            g_atomic_pointer_get(sys_key + target_system));
    g_rw_lock_reader_unlock(&parameter_hash_table_lock);

    for (gsize j = 0; j < 10; j++)
    {
        for (gsize i = 0; i < PARAM_COUNT; i++)
        {
            g_mutex_lock(&parameter_mutex[target_system]);
            gchar param_id_stx = current_parameter[i].param_id[0];
            g_mutex_unlock(&parameter_mutex[target_system]);

            if (0 == param_id_stx)
            {
                send_param_request_read(target_system, target_component, i);
                g_usleep(100000);
            }
        }

        g_usleep(100000);
    }

    g_mutex_lock(&parameter_mutex[target_system]);
    for (gsize i = 0; i < PARAM_COUNT; i++)
    {
        if (0 == current_parameter[i].param_id[0])
        {
            g_message("Fetch all parameter FAILED!");
        }
    }
    g_mutex_unlock(&parameter_mutex[target_system]);

    g_message("Fetch all parameter SUCCEED!");

    return NULL;
}

gpointer named_val_float_handle_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;

    // wait for named_val_float_queue ready
    while (NULL == g_atomic_pointer_get(
                       named_val_float_queue + my_target_system))
    {
        g_usleep(100);
    }

    mavlink_named_value_float_t *my_named_value_float =
        named_val_float_queue_pop(my_target_system);

    while (TRUE)
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
            g_usleep(10000);
        }

        my_named_value_float =
            named_val_float_queue_pop(my_target_system);
    }

    return NULL;
}

gpointer vehicle_data_update_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;

    while (NULL == g_atomic_pointer_get(vehicle_data_array + my_target_system))
    {
        g_usleep(10);
    }

    Vehicle_Data_t *my_vehicle_data = g_atomic_pointer_get(vehicle_data_array + my_target_system);
    g_assert(NULL != my_vehicle_data);

    Mavlink_Messages_t *my_mavlink_message = message_queue_pop(my_target_system);

    while (TRUE)
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

            default:
                break;
            }

            g_mutex_unlock(&vehicle_data_mutex[my_target_system]);
        }
        else
        {
            g_usleep(100);
        }
        my_mavlink_message = message_queue_pop(my_target_system);
    }

    return NULL;
}

gpointer db_update_worker(gpointer data)
{
    g_assert(NULL != data);

    guint8 my_target_system = *(guint8 *)data;

    as_sql_check_vechle_table(my_target_system);

    Vehicle_Data_t *my_vehicle_data = g_atomic_pointer_get(vehicle_data_array + my_target_system);
    g_assert(NULL != my_vehicle_data);

    while (TRUE)
    {
        g_mutex_lock(&vehicle_data_mutex[my_target_system]);

        as_sql_insert_vechle_table(my_target_system, my_vehicle_data);

        g_mutex_unlock(&vehicle_data_mutex[my_target_system]);

        g_usleep(10000);
    }

    return NULL;
}