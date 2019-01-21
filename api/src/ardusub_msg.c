#include "../inc/ardusub_msg.h"

/**
 * @brief Handle Messages, parse msg_tmp, if the msg_tmp parse successful, 
 * then decode the message and pass the value to message_hash_table[$sysid].
 * 
 * @param msg_tmp :buff that contains raw data from UDP 
 * @param bytes_read :buff lenth
 * @return guint8 :FALSE if never parse successful, msgid if parse successful
 */
guint8 as_handle_messages(gchar *msg_tmp, gsize bytes_read)
{
    guint8 target_system;
    guint8 target_autopilot;

    Mavlink_Messages_t *current_messages = NULL;
    Mavlink_Parameter_t *current_parameter = NULL;

    mavlink_message_t message;
    mavlink_status_t status;
    gboolean msgReceived = FALSE;

    for (gsize i = 0; i < (bytes_read < MAX_BYTES ? bytes_read : MAX_BYTES) && FALSE == msgReceived; i++)
    {
        msgReceived =
            mavlink_parse_char(MAVLINK_COMM_1, msg_tmp[i], &message, &status);
    }

    if (FALSE == msgReceived)
    {
        return msgReceived;
    }

    // NOTE: this doesn't handle multiple compid for one sysid.
    target_system = message.sysid;
    target_autopilot = message.compid;

    g_mutex_lock(&message_mutex[target_system]);

    if (SYS_UN_INIT == g_atomic_int_get(vehicle_status + target_system)) // find new system
    {
        g_atomic_int_set(vehicle_status + target_system, SYS_INITIATING);
        current_messages = g_new0(Mavlink_Messages_t, 1);
        current_parameter = g_new0(Mavlink_Parameter_t, PARAM_COUNT);
        GSocket *current_target_socket = g_new0(GSocket, 1);

        if ((NULL == current_messages) ||
            (NULL == current_parameter) ||
            (NULL == current_target_socket))
        {
            g_error("Out of memory!");
        }

        // add system to hash table if sysid NOT exsit in hash table's key set
        g_message("Found a new system: %d", target_system);
        g_message("adding...");
        as_system_add(target_system, target_autopilot,
                      current_messages,
                      current_parameter,
                      current_target_socket);
        g_atomic_int_set(vehicle_status + target_system, SYS_DISARMED);
        g_message("New system added: %d", target_system);
    }
    else
    {
        // lock the hash table
        g_rw_lock_reader_lock(&message_hash_table_lock);
        g_rw_lock_reader_lock(&parameter_hash_table_lock);

        // set current message parameter and target_socket.
        current_messages =
            g_hash_table_lookup(message_hash_table,
                                sys_key[target_system]);

        current_parameter =
            g_hash_table_lookup(parameter_hash_table,
                                sys_key[target_system]);

        // unlock the message hash table
        g_rw_lock_reader_unlock(&message_hash_table_lock);
        g_rw_lock_reader_unlock(&parameter_hash_table_lock);
    }

    g_assert(current_messages != NULL);
    g_assert(current_parameter != NULL);

    current_messages->sysid = target_system;
    current_messages->compid = target_autopilot;

    as_handle_message_id(message,
                         current_messages,
                         current_parameter);

    g_mutex_unlock(&message_mutex[target_system]);

    return message.msgid;
}

void as_handle_message_id(mavlink_message_t message,
                          Mavlink_Messages_t *current_messages,
                          Mavlink_Parameter_t *current_parameter)
{
    guint8 target_system = current_messages->sysid;
    current_messages->msg_id = message.msgid;
    gboolean queue_push = FALSE;

    // Handle Message ID
    switch (message.msgid)
    {

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // printf("MAVLINK_MSG_ID_HEARTBEAT\n");
        mavlink_msg_heartbeat_decode(&message, &(current_messages->heartbeat));

        /* Queries the system monotonic time. in microseconds (gint64) */
        // g_get_monotonic_time();
        /* https://developer.gnome.org/glib/stable/glib-Date-and-Time-Functions.html#g-get-monotonic-time */

        current_messages->time_stamps.heartbeat = g_get_monotonic_time();

        /* MAY THE SOURCE BE WITH YOU */
        // printf("timestamp:%I64d\n", current_messages->time_stamps.heartbeat);
        // printf("Received message from sys:%d|comp:%d\n",
        //        current_messages->sysid, current_messages->compid);
        // printf("type:%d, autopilot:%d, base_mode:%d, custom_mode:%d, system_status:%d, mavlink_version:%d \n",
        //        current_messages->heartbeat.type, current_messages->heartbeat.autopilot,
        //        current_messages->heartbeat.base_mode, current_messages->heartbeat.custom_mode,
        //        current_messages->heartbeat.system_status, current_messages->heartbeat.mavlink_version);

        // g_message("heartbeat msg from system:%d", current_messages->sysid);

        // send heartbeat
        if (TRUE == g_atomic_int_get((volatile gint *)(udp_write_ready + target_system)))
        {
            send_heartbeat(target_system);
        }
        queue_push = TRUE;

        break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        // printf("MAVLINK_MSG_ID_SYS_STATUS\n");
        mavlink_msg_sys_status_decode(&message, &(current_messages->sys_status));
        // printf("SYS_STATUS: erros_comm %d %d \n", current_messages->sys_status.voltage_battery,
        //    current_messages->sys_status.current_battery);
        current_messages->time_stamps.sys_status = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }

    case MAVLINK_MSG_ID_PING:
    {
        printf("MAVLINK_MSG_ID_PING\n");
        mavlink_msg_ping_decode(&message, &(current_messages->ping));
        //TODO: fix this
        printf("PING: time_usec:%I64u, seq:%d, target_system:%d, target_component:%d\n",
               current_messages->ping.time_usec, current_messages->ping.seq,
               current_messages->ping.target_system, current_messages->ping.target_component);

        break;
    }

    case MAVLINK_MSG_ID_BATTERY_STATUS:
    {
        // printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
        mavlink_msg_battery_status_decode(&message, &(current_messages->battery_status));
        current_messages->time_stamps.battery_status = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }

    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        // printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
        mavlink_msg_radio_status_decode(&message, &(current_messages->radio_status));
        current_messages->time_stamps.radio_status = g_get_monotonic_time();
        break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
        mavlink_msg_local_position_ned_decode(&message, &(current_messages->local_position_ned));
        current_messages->time_stamps.local_position_ned = g_get_monotonic_time();
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
        mavlink_msg_global_position_int_decode(&message, &(current_messages->global_position_int));
        current_messages->time_stamps.global_position_int = g_get_monotonic_time();
        break;
    }

    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
    {
        //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
        mavlink_msg_position_target_local_ned_decode(&message, &(current_messages->position_target_local_ned));
        current_messages->time_stamps.position_target_local_ned = g_get_monotonic_time();
        break;
    }

    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
    {
        // printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
        mavlink_msg_position_target_global_int_decode(&message, &(current_messages->position_target_global_int));
        current_messages->time_stamps.position_target_global_int = g_get_monotonic_time();
        break;
    }

    case MAVLINK_MSG_ID_HIGHRES_IMU:
    {
        // printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
        mavlink_msg_highres_imu_decode(&message, &(current_messages->highres_imu));
        current_messages->time_stamps.highres_imu = g_get_monotonic_time();
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE:
    {
        // printf("MAVLINK_MSG_ID_ATTITUDE\n");
        mavlink_msg_attitude_decode(&message, &(current_messages->attitude));
        current_messages->time_stamps.attitude = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }

    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
    {
        //printf("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW\n");
        mavlink_msg_servo_output_raw_decode(&message, &(current_messages->servo_output_raw));
        current_messages->time_stamps.servo_output_raw = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        // printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
        mavlink_msg_command_ack_decode(&message, &(current_messages->command_ack));
        current_messages->time_stamps.command_ack = g_get_monotonic_time();
        // printf("Command_ACK, command:%d, result:%d. \n", current_messages->command_ack.command,
        //        current_messages->command_ack.result);
        break;
    }
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
    {
        // printf("MAVLINK_MSG_ID_NAMED_VALUE_FLOA\n");
        mavlink_msg_named_value_float_decode(&message, &(current_messages->named_value_float));
        current_messages->time_stamps.named_value_float = g_get_monotonic_time();
        // printf(current_messages->named_value_float.name);
        // printf(": %f \n", current_messages->named_value_float.value);
        named_val_float_queue_push(target_system, current_messages);
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD:
    {
        // printf("MAVLINK_MSG_ID_VFR_HUD\n");
        mavlink_msg_vfr_hud_decode(&message, &(current_messages->vfr_hud));
        current_messages->time_stamps.vfr_hud = g_get_monotonic_time();
        // printf("heading:%d\n", current_messages->vfr_hud.heading);
        break;
    }
    case MAVLINK_MSG_ID_POWER_STATUS:
    {
        // printf("MAVLINK_MSG_ID_POWER_STATUS\n");
        mavlink_msg_power_status_decode(&message, &(current_messages->power_status));
        current_messages->time_stamps.power_status = g_get_monotonic_time();
        // printf("Vcc(5V rail voltage in mV):%d, Vservo(servo rail voltage in mV):%d, "
        // "power supply status flags:%d.\n", current_messages->power_status.Vcc,
        // current_messages->power_status.Vservo, current_messages->power_status.flags);
        queue_push = TRUE;

        break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME:
    {
        // printf("MAVLINK_MSG_ID_SYSTEM_TIME\n");
        mavlink_msg_system_time_decode(&message, &(current_messages->system_time));
        current_messages->time_stamps.system_time = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    {
        // printf("MAVLINK_MSG_ID_MISSION_CURRENT\n");
        mavlink_msg_mission_current_decode(&message, &(current_messages->mission_current));
        current_messages->time_stamps.mission_current = g_get_monotonic_time();
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
        // printf("MAVLINK_MSG_ID_GPS_RAW_INT\n");
        mavlink_msg_gps_raw_int_decode(&message, &(current_messages->gps_raw_int));
        current_messages->time_stamps.gps_raw_int = g_get_monotonic_time();
        break;
    }
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
    {
        // printf("MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT\n");
        mavlink_msg_nav_controller_output_decode(&message, &(current_messages->nav_controller_output));
        current_messages->time_stamps.nav_controller_output = g_get_monotonic_time();
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS:
    {
        // printf("MAVLINK_MSG_ID_RC_CHANNELS\n");
        mavlink_msg_rc_channels_decode(&message, &(current_messages->rc_channels));
        current_messages->time_stamps.rc_channels = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }
    case MAVLINK_MSG_ID_VIBRATION:
    {
        // printf("MAVLINK_MSG_ID_VIBRATION\n");
        mavlink_msg_vibration_decode(&message, &(current_messages->vibration));
        current_messages->time_stamps.vibration = g_get_monotonic_time();
        break;
    }
    case MAVLINK_MSG_ID_RAW_IMU:
    {
        // printf("MAVLINK_MSG_ID_RAW_IMU\n");
        mavlink_msg_raw_imu_decode(&message, &(current_messages->raw_imu));
        current_messages->time_stamps.raw_imu = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
    {
        // printf("MAVLINK_MSG_ID_SCALED_PRESSURE\n");
        mavlink_msg_scaled_pressure_decode(&message, &(current_messages->scaled_pressure));
        current_messages->time_stamps.scaled_pressure = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU2:
    {
        // printf("MAVLINK_MSG_ID_SCALED_IMU2\n");
        mavlink_msg_scaled_imu2_decode(&message, &(current_messages->scaled_imu2));
        current_messages->time_stamps.scaled_imu2 = g_get_monotonic_time();
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE2:
    {
        // printf("MAVLINK_MSG_ID_SCALED_PRESSURE2\n");
        mavlink_msg_scaled_pressure2_decode(&message, &(current_messages->scaled_pressure2));
        current_messages->time_stamps.scaled_pressure2 = g_get_monotonic_time();
        queue_push = TRUE;

        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    {
        // printf("MAVLINK_MSG_ID_RC_CHANNELS_RAW\n");
        mavlink_msg_rc_channels_raw_decode(&message, &(current_messages->rc_channels_raw));
        current_messages->time_stamps.rc_channels_raw = g_get_monotonic_time();
        break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT:
    {
        // printf("MAVLINK_MSG_ID_STATUSTEXT\n");
        mavlink_msg_statustext_decode(&message, &(current_messages->statustext));
        current_messages->time_stamps.statustext = g_get_monotonic_time();
        statustex_queue_push(target_system, current_messages);
        // printf("severity: %d, statustext: ", current_messages->statustext.severity);
        // printf(current_messages->statustext.text);
        // printf("\n");
        break;
    }
    case MAVLINK_MSG_ID_PARAM_VALUE:
    {
        // printf("MAVLINK_MSG_ID_PARAM_VALUE\n");
        mavlink_msg_param_value_decode(&message, &(current_messages->param_value));
        current_messages->time_stamps.param_value = g_get_monotonic_time();

        guint16 _param_index = current_messages->param_value.param_index;

        if (_param_index > PARAM_COUNT - 1)
        {
            // param_index out of range!
            // g_warning("param_index out of range! param_index:%s, param_index:%d, PARAM_COUNT:%d\n",
            //           current_messages->param_value.param_id,
            //           _param_index, PARAM_COUNT);

            break;
        }
        else
        {
            g_mutex_lock(&parameter_mutex[target_system]);

            strcpy(current_parameter[_param_index].param_id,
                   current_messages->param_value.param_id);

            current_parameter[_param_index].param_type =
                current_messages->param_value.param_type;

            current_parameter[_param_index].param_value.param_float =
                current_messages->param_value.param_value;

            g_mutex_unlock(&parameter_mutex[target_system]);
        }

        // printf("param_id:%s, param_value:%d, param_type:%d, param_count:%d, param_index:%d\n",
        //        current_messages->param_value.param_id, current_messages->param_value.param_value,
        //        current_messages->param_value.param_type, current_messages->param_value.param_count,
        //        current_messages->param_value.param_index);

        break;
    }
    default:
    {
        g_message("Warning, did not handle message id %i\n", message.msgid);
        break;
    }

    } // end: switch msgid

    if (TRUE == queue_push)
    {
        message_queue_push(target_system, current_messages);
    }
}
