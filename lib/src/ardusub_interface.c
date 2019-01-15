#include "../inc/ardusub_interface.h"

void as_api_init(char *p_subnet_address)
{
    static gboolean as_init_status;

    //! only init once
    if (TRUE != as_init_status)
    {
        // initialize attributes

        if (NULL == p_subnet_address)
        {
            subnet_address = SUBNET_ADDRESS;
        }
        else
        {
            subnet_address = p_subnet_address;
        }

        // serial_port = serial_port_; // serial port management object

        message_hash_table = g_hash_table_new(g_int_hash, g_int_equal);
        parameter_hash_table = g_hash_table_new(g_int_hash, g_int_equal);
        target_socket_hash_table = g_hash_table_new(g_int_hash, g_int_equal);
        manual_control_table = g_hash_table_new(g_int_hash, g_int_equal);

        as_udp_read_init();

        as_init_status = TRUE;

        g_thread_new("as_api_main", &as_api_run, NULL);
    }
}

void as_api_deinit()
{
    ;
}

gpointer as_api_run(gpointer data)
{
    g_assert(NULL == data);

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);
    g_main_loop_unref(loop);

    return NULL;
}

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

    as_handle_messages(msg_tmp, bytes_read);

    return TRUE;
}

void as_udp_read_init()
{
    GSocket *socket_udp_read;
    GError *error = NULL;
    GSocketAddress *gsock_addr_read = G_SOCKET_ADDRESS(
        // g_inet_socket_address_new(g_inet_address_new_from_string("192.168.1.120"), 14551));
        // g_inet_socket_address_new(g_inet_address_new_from_string("192.168.2.1"), 14551));
        g_inet_socket_address_new(g_inet_address_new_any(G_SOCKET_FAMILY_IPV4), 14551));
    socket_udp_read = g_socket_new(G_SOCKET_FAMILY_IPV4, G_SOCKET_TYPE_DATAGRAM, G_SOCKET_PROTOCOL_UDP, &error);

    /* don't forget to check for errors */
    if (error != NULL)
    {
        g_error(error->message);
    }

    g_socket_bind(socket_udp_read, gsock_addr_read, TRUE, &error);

    /* don't forget to check for errors */
    if (error != NULL)
    {
        g_error(error->message);
    }

    gint fd = g_socket_get_fd(socket_udp_read);

    GIOChannel *channel = g_io_channel_win32_new_socket(fd);
    g_io_channel_set_encoding(channel, NULL, &error);
    g_io_add_watch(channel, G_IO_IN, (GIOFunc)udp_read_callback, NULL);
}

void as_udp_write_init(guint8 sysid, GSocket *p_target_socket)
{
    g_assert(p_target_socket != NULL);

    GError *error = NULL;
    gchar inet_address_string[16] = {0};

    g_snprintf(inet_address_string, 16, "%s%d", subnet_address, sysid + 1);

    GSocketAddress *gsock_addr_write = G_SOCKET_ADDRESS(
        g_inet_socket_address_new(g_inet_address_new_from_string(inet_address_string), 14551));

    *p_target_socket = *g_socket_new(G_SOCKET_FAMILY_IPV4, G_SOCKET_TYPE_DATAGRAM, G_SOCKET_PROTOCOL_UDP, &error);

    /* don't forget to check for errors */
    if (error != NULL)
    {
        g_error(error->message);
    }

    g_socket_connect(p_target_socket, gsock_addr_write, NULL, &error);

    /* don't forget to check for errors */
    if (error != NULL)
    {
        g_error(error->message);
    }

    g_atomic_int_set((volatile gint *)(udp_write_ready + sysid), TRUE);
}

void as_system_init(guint8 target_system, guint8 target_autopilot,
                    Mavlink_Messages_t *current_messages,
                    Mavlink_Parameter_t *current_parameter,
                    GSocket *current_target_socket)
{
    g_assert(current_messages != NULL);
    g_assert(current_parameter != NULL);
    g_assert(current_target_socket != NULL);

    sys_count++;

    guint8 *p_sysid = g_new0(guint8, 1);
    *p_sysid = target_system;

    // lock the hash table
    g_rw_lock_writer_lock(&message_hash_table_lock);
    g_rw_lock_writer_lock(&parameter_hash_table_lock);
    g_rw_lock_writer_lock(&target_socket_hash_table_lock);

    g_hash_table_insert(message_hash_table, p_sysid, current_messages);

    g_hash_table_insert(parameter_hash_table, p_sysid, current_parameter);

    as_udp_write_init(target_system, current_target_socket); // init write socket for new system
    g_hash_table_insert(target_socket_hash_table, p_sysid, current_target_socket);

    mavlink_manual_control_t *p_manual_control = g_new0(mavlink_manual_control_t, 1);
    p_manual_control->z = 500; // 500 is z axis zero leval
    g_hash_table_insert(manual_control_table, p_sysid, p_manual_control);

    // unlock the message hash table
    g_rw_lock_writer_unlock(&message_hash_table_lock);
    g_rw_lock_writer_unlock(&parameter_hash_table_lock);
    g_rw_lock_writer_unlock(&target_socket_hash_table_lock);

    statustex_queue[target_system] = g_async_queue_new();

    sys_key[target_system] = p_sysid;

    if (-1 == as_request_full_parameters(
                  target_system, target_autopilot))
    {
        g_message("faild to request full parameters!\n");
    }

    // init manual_control_worker thread
    g_thread_new("manual_control_worker", &manual_control_worker, p_sysid);
}

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
            send_udp_message(my_target_system, &message);

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

        // add system to hash table if sysid NOT exsit in hash table's key set
        as_system_init(target_system, target_autopilot,
                       current_messages,
                       current_parameter,
                       current_target_socket);
        g_atomic_int_set(vehicle_status + target_system, SYS_DISARMED);
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
        // printf("timestamp:%d\n", current_messages->time_stamps.heartbeat);
        // printf("Received message from sys:%d|comp:%d\n",
        //        current_messages->sysid, current_messages->compid);
        // printf("type:%d, autopilot:%d, base_mode:%d, custom_mode:%d, system_status:%d, mavlink_version:%d \n",
        //        current_messages->heartbeat.type, current_messages->heartbeat.autopilot,
        //        current_messages->heartbeat.base_mode, current_messages->heartbeat.custom_mode,
        //        current_messages->heartbeat.system_status, current_messages->heartbeat.mavlink_version);

        g_message("heartbeat msg from system:%d", current_messages->sysid);

        // send heartbeat
        if (TRUE == g_atomic_int_get((volatile gint *)(udp_write_ready + target_system)))
        {
            send_heartbeat(target_system, current_messages);
        }

        break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        // printf("MAVLINK_MSG_ID_SYS_STATUS\n");
        mavlink_msg_sys_status_decode(&message, &(current_messages->sys_status));
        // printf("SYS_STATUS: erros_comm %d %d \n", current_messages->sys_status.voltage_battery,
        //    current_messages->sys_status.current_battery);
        current_messages->time_stamps.sys_status = g_get_monotonic_time();
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
        break;
    }

    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
    {
        //printf("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW\n");
        mavlink_msg_servo_output_raw_decode(&message, &(current_messages->servo_output_raw));
        current_messages->time_stamps.servo_output_raw = g_get_monotonic_time();
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
        break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME:
    {
        // printf("MAVLINK_MSG_ID_SYSTEM_TIME\n");
        mavlink_msg_system_time_decode(&message, &(current_messages->system_time));
        current_messages->time_stamps.system_time = g_get_monotonic_time();
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
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
    {
        // printf("MAVLINK_MSG_ID_SCALED_PRESSURE\n");
        mavlink_msg_scaled_pressure_decode(&message, &(current_messages->scaled_pressure));
        current_messages->time_stamps.scaled_pressure = g_get_monotonic_time();
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
            g_warning("param_index out of range! param_index:%s, param_index:%d, PARAM_COUNT:%d\n",
                      current_messages->param_value.param_id,
                      _param_index, PARAM_COUNT);

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
}

void as_api_manual_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, ...)
{
    uint8_t sys_id = 1;

    if (sys_count > 1)
    {
        va_list ap;
        va_start(ap, 1);
        sys_id = va_arg(ap, int);
        va_end(ap);
    }

    // arm first
    if (SYS_ARMED != g_atomic_int_get(vehicle_status + sys_id))
    {
        return;
    }

    g_rw_lock_reader_lock(&manual_control_hash_table_lock);
    mavlink_manual_control_t *p_manual_control =
        g_hash_table_lookup(manual_control_table, sys_key[sys_id]);
    g_rw_lock_reader_unlock(&manual_control_hash_table_lock);

    g_mutex_lock(&manual_control_mutex[sys_id]); // lock
    p_manual_control->x = x;
    p_manual_control->y = y;
    p_manual_control->z = z;
    p_manual_control->r = r;
    p_manual_control->buttons = buttons;
    g_mutex_unlock(&manual_control_mutex[sys_id]); // unlock
}

Mavlink_Messages_t *as_get_meaasge(uint8_t sysid)
{
    gpointer system_key_ = g_atomic_pointer_get(sys_key + sysid);
    g_assert(NULL != system_key_);

    g_rw_lock_reader_lock(&message_hash_table_lock);
    Mavlink_Messages_t *p_message = g_hash_table_lookup(message_hash_table, system_key_);
    g_rw_lock_reader_unlock(&message_hash_table_lock);

    g_assert(NULL != p_message);

    return p_message;
}

int as_api_check_vehicle(uint8_t sysid)
{
    if ((NULL == g_atomic_pointer_get(sys_key + sysid)) ||
        (SYS_DISARMED != g_atomic_int_get(vehicle_status + sysid)))
    {
        return 0;
    }
    else
    {
        return !0;
    }
}

void send_heartbeat(guint8 target_system,
                    Mavlink_Messages_t *current_messages)
{
    mavlink_message_t message;
    mavlink_heartbeat_t hb;

    // TODO: change this value fix value

    hb.type = current_messages->heartbeat.type;
    hb.autopilot = current_messages->heartbeat.autopilot;
    hb.base_mode = current_messages->heartbeat.base_mode;
    hb.custom_mode = current_messages->heartbeat.custom_mode;
    hb.system_status = current_messages->heartbeat.system_status;
    hb.mavlink_version = current_messages->heartbeat.mavlink_version;

    mavlink_msg_heartbeat_encode(255, 1, &message, &hb);

    send_udp_message(target_system, &message);

    // g_print("heart breat sended!\n");
}

void do_set_servo(guint8 target_system,
                  guint8 target_autopilot,
                  gfloat servo_no, gfloat pwm)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    mavlink_command_long_t cmd_long;
    cmd_long.target_system = target_system;
    cmd_long.target_component = target_autopilot;
    cmd_long.command = MAV_CMD_DO_SET_SERVO;
    cmd_long.confirmation = 0;
    cmd_long.param1 = servo_no;
    cmd_long.param2 = pwm;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_command_long_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &cmd_long);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    send_udp_message(target_system, &message);

    // printf("do_set_servo msg wrote!");
}

void do_motor_test(guint8 target_system,
                   guint8 target_autopilot,
                   gfloat motor_no, gfloat pwm)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    mavlink_command_long_t cmd_long;
    cmd_long.target_system = target_system;
    cmd_long.target_component = target_autopilot;
    cmd_long.command = MAV_CMD_DO_MOTOR_TEST;
    cmd_long.confirmation = 0;
    cmd_long.param1 = motor_no - 1;
    cmd_long.param2 = MOTOR_TEST_THROTTLE_PWM;
    cmd_long.param3 = pwm;
    cmd_long.param4 = 10;
    cmd_long.param5 = 8;
    cmd_long.param6 = MOTOR_TEST_ORDER_DEFAULT;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_command_long_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &cmd_long);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    // int len = write_message(message);
    printf("do_motor_test msg wrote!");
    // check the write
}

void do_set_mode(control_mode_t mode, guint8 target_system)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    mavlink_set_mode_t set_mode;
    set_mode.target_system = target_system;
    set_mode.base_mode = 209; //81
    set_mode.custom_mode = mode;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_set_mode_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &set_mode);
    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    // int len = write_message(message);
    printf("do_set_mode msg wrote!");
    // check the write
}

gint as_request_full_parameters(guint8 target_system, guint8 target_component)
{
    //TODO: request full parameters
    send_param_request_list(target_system, target_component); // no guarantee
    g_usleep(100000);

    g_mutex_lock(&parameter_mutex[target_system]);
    // creat a new thread to do this
    g_mutex_unlock(&parameter_mutex[target_system]);

    return 0;
}

void send_param_request_read(guint8 target_system, guint8 target_component, gint16 param_index)
{
    if (-1 == param_index)
    {
        return;
    }

    mavlink_param_request_read_t prr = {0};

    prr.target_system = target_system;
    prr.target_component = target_component;
    prr.param_index = param_index;
    strcpy(prr.param_id, ""); // if param_index != -1, param_id is not useed

    mavlink_message_t message;

    mavlink_msg_param_request_read_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &prr);

    send_udp_message(target_system, &message);
}

void send_param_request_list(guint8 target_system, guint8 target_autopilot)
{
    mavlink_param_request_list_t param_list = {0};

    param_list.target_system = target_system;
    param_list.target_component = target_autopilot;

    // Encode
    mavlink_message_t message;

    //companion_id STATION_COMID
    mavlink_msg_param_request_list_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &param_list);

    // Send
    send_udp_message(target_system, &message);

    // g_message("param_request_list msg wrote!");
}

void send_rc_channels_override(guint8 target_system, guint8 target_autopilot,
                               uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4,
                               uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_rc_channels_override_t rc_channels_override;
    rc_channels_override.target_system = target_system;
    rc_channels_override.target_component = target_autopilot;
    rc_channels_override.chan1_raw = ch1;
    rc_channels_override.chan2_raw = ch2;
    rc_channels_override.chan3_raw = ch3;
    rc_channels_override.chan4_raw = ch4;
    rc_channels_override.chan5_raw = ch5;
    rc_channels_override.chan6_raw = ch6;
    rc_channels_override.chan7_raw = ch7;
    rc_channels_override.chan8_raw = ch8;
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_rc_channels_override_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message,
                                            &rc_channels_override);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    // int len = write_message(message);
    printf("send_rc_channels_override msg wrote!");
    // check the write
}

// ------------------------------------------------------------------------------
//   ARM the vehicle
// ------------------------------------------------------------------------------
void as_api_vehicle_arm(guint8 target_system, guint8 target_autopilot)
{
    mavlink_command_long_t cmd = {0};

    cmd.target_system = target_system;
    cmd.target_component = target_autopilot;
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 1.0F;

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &cmd);

    g_rw_lock_reader_lock(&manual_control_hash_table_lock);
    mavlink_manual_control_t *p_manual_control =
        g_hash_table_lookup(manual_control_table, sys_key[target_system]);
    g_rw_lock_reader_unlock(&manual_control_hash_table_lock);

    g_assert(NULL != p_manual_control);

    g_mutex_lock(&manual_control_mutex[target_system]); // lock
    // clear manual_control value
    p_manual_control->x = 0;
    p_manual_control->y = 0;
    p_manual_control->z = 500;
    p_manual_control->r = 0;
    p_manual_control->buttons = 0;
    g_mutex_unlock(&manual_control_mutex[target_system]); // unlock

    g_atomic_int_set(vehicle_status + target_system, SYS_ARMED);

    // Send the message
    send_udp_message(target_system, &message);
}

// ------------------------------------------------------------------------------
//   DISARM the vehicle
// ------------------------------------------------------------------------------
void as_api_vehicle_disarm(guint8 target_system, guint8 target_autopilot)
{
    mavlink_command_long_t cmd = {0};

    cmd.target_system = target_system;
    cmd.target_component = target_autopilot;
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 0.0F;

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &cmd);

    // Send the message
    send_udp_message(target_system, &message);

    g_atomic_int_set(vehicle_status + target_system, SYS_DISARMED);

    g_rw_lock_reader_lock(&manual_control_hash_table_lock);
    mavlink_manual_control_t *p_manual_control =
        g_hash_table_lookup(manual_control_table, sys_key[target_system]);
    g_rw_lock_reader_unlock(&manual_control_hash_table_lock);

    g_assert(NULL != p_manual_control);

    g_mutex_lock(&manual_control_mutex[target_system]); // lock
    // clear manual_control value
    p_manual_control->x = 0;
    p_manual_control->y = 0;
    p_manual_control->z = 500;
    p_manual_control->r = 0;
    p_manual_control->buttons = 0;
    g_mutex_unlock(&manual_control_mutex[target_system]); // unlock
}

void send_udp_message(guint8 target_system, mavlink_message_t *message)
{
    guint msg_len;
    gchar msg_buf[MAX_BYTES];
    GError *error = NULL;

    gpointer system_key_ = g_atomic_pointer_get(sys_key + target_system);

    g_assert(NULL != system_key_);
    g_assert(NULL != message);

    g_assert(TRUE == g_atomic_int_get((volatile gint *)(udp_write_ready + target_system)));

    g_rw_lock_reader_lock(&target_socket_hash_table_lock);
    GSocket *target_socket = g_hash_table_lookup(target_socket_hash_table,
                                                 system_key_);
    g_rw_lock_reader_unlock(&target_socket_hash_table_lock);

    g_assert(NULL != target_socket);

    // Translate message to buffer
    msg_len = mavlink_msg_to_send_buffer((uint8_t *)msg_buf, message);

    // Send
    g_socket_send(target_socket, msg_buf, msg_len, NULL, &error);

    /* don't forget to check for errors */
    if (error != NULL)
    {
        g_error(error->message);
    }
}

//! NULL-able return value
mavlink_statustext_t *as_api_statustex_queue_pop(uint8_t target_system)
{
    return statustex_queue_pop(target_system);
}

int as_api_statustex_cpunt(uint8_t target_system)
{
    return g_async_queue_length(statustex_queue[target_system]);
}

mavlink_statustext_t *statustex_queue_pop(guint8 target_system)
{
    static mavlink_statustext_t *last_statustex;

    if (NULL != last_statustex)
    {
        // free last statustex after pop
        g_free(last_statustex);
    }

    last_statustex = g_async_queue_try_pop(statustex_queue[target_system]);

    return last_statustex;
}

void statustex_queue_push(guint8 target_system,
                          Mavlink_Messages_t *current_messages)
{
    // TODO: fix this ??? need fix ???
    g_assert(NULL != current_messages);

    if (g_async_queue_length(statustex_queue[target_system]) > MAX_STATUSTEX)
    {
        statustex_queue_pop(target_system);
    }

    gpointer statustex_p = g_memdup(&current_messages->statustext,
                                    sizeof(mavlink_statustext_t));

    if (NULL == statustex_p)
    {
        g_error("Out of memory!");
    }

    g_async_queue_push(statustex_queue[target_system], // queue
                       statustex_p);
}
