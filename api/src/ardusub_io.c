/**
 * @file ardusub_io.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-02-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#define G_LOG_DOMAIN "[ardusub io        ]"

#include "../inc/ardusub_io.h"
#include "../inc/ardusub_msg.h"

/**
 * @brief udp read init
 * 
 */
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

#ifdef _WIN32
    GIOChannel *channel = g_io_channel_win32_new_socket(fd);
#else
    GIOChannel *channel = g_io_channel_unix_new(fd);
#endif

    g_io_channel_set_encoding(channel, NULL, &error);
    g_io_add_watch(channel, G_IO_IN, (GIOFunc)udp_read_callback, NULL);
}

/**
 * @brief serial read init
 * 
 */
#ifndef NO_SERISL
void as_serial_read_init()
{
    // prepare serial_write_buf_queue
    for (gint i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++)
    {
        serial_write_buf_queue[i] = g_async_queue_new();
    }

    struct sp_port **serial_port_list;
    enum sp_return sp_result;

    // Enumerating the serial ports...
    sp_result = sp_list_ports(&serial_port_list);

    if (SP_OK == sp_result)
    {
        // count serial port
        gsize serial_count = 0;
        while (NULL != serial_port_list[serial_count])
        {
            serial_count++;
        }

        g_message("got %lu ports.", serial_count);

        // check for Pixhawk device
        gint vid, pid;
        for (gsize i = 0; i < serial_count; i++)
        {
            // check USB serial port adapter
            if (SP_TRANSPORT_USB != sp_get_port_transport(serial_port_list[i]))
            {
                continue;
            }

            sp_result = sp_get_port_usb_vid_pid(serial_port_list[i], &vid, &pid);

            // Pixhawk device vid == 9900, pid == 17
            if (vid == 9900 && pid == 17)
            {
                g_print("Pixhawk device.\n");

                // open serial port
                if (SP_OK != sp_open(serial_port_list[i], SP_MODE_READ_WRITE))
                {
                    g_error("port open faild!");
                }

                g_thread_new("serial_port_read_write_worker",
                             &serial_port_read_write_worker,
                             serial_port_list[i]);
            }
            // ArduPilot Pixhawk1 device vid == 483, pid == 5740
            else if (vid == 1155 && pid == 22336)
            {
                g_message("ArduPilot Pixhawk1 device vid == %d, pid == %d", vid, pid);
                // g_message("pass ArduPilot Pixhawk1 device.");
                // continue;
                // open serial port
                if (SP_OK != sp_open(serial_port_list[i], SP_MODE_READ_WRITE))
                {
                    g_error("port open faild!");
                }

                g_thread_new("serial_port_read_write_worker",
                             &serial_port_read_write_worker,
                             serial_port_list[i]);
            }
            // Silicon Labs CP210x USB to UART Bridge vid == 4292, pid == 60000
            else if (vid == 4292 && pid == 60000)
            {
                g_message("Silicon Labs CP210x USB to UART Bridge vid == %d, pid == %d", vid, pid);
                // open serial port
                if (SP_OK != sp_open(serial_port_list[i], SP_MODE_READ_WRITE))
                {
                    g_error("port open faild!");
                }

                g_thread_new("serial_port_read_write_worker",
                             &serial_port_read_write_worker,
                             serial_port_list[i]);
            }
            else
            {
                g_message("UNKOWN USB to UART Bridge vid == %d, pid == %d", vid, pid);
            }
        }
    }

    // sp_free_port_list(serial_port_list);
}
#endif

/**
 * @brief serial write init
 * 
 */
void as_serial_write_init()
{
    // do nothing
    ;
}

/**
 * @brief udp write init
 * 
 * @param sysid 
 * @param p_target_socket 
 */
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
}

/**
 * @brief find new system
 * 
 * @param message 
 * @param targer_serial_chan 
 * @return gboolean 
 */
gboolean as_find_new_system(mavlink_message_t message, guint8 *targer_serial_chan)
{
    guint8 target_system;
    guint8 target_autopilot;

    gboolean new_system = FALSE;

    // NOTE: this doesn't handle multiple compid for one sysid.
    target_system = message.sysid;
    target_autopilot = message.compid;

    g_mutex_lock(&message_mutex[target_system]);

    if (SYS_UN_INIT == g_atomic_int_get(vehicle_status + target_system)) // find new system
    {
        g_atomic_int_set(vehicle_status + target_system, SYS_INITIATING);
        Mavlink_Messages_t *current_messages = g_new0(Mavlink_Messages_t, 1);
        Mavlink_Parameter_t *current_parameter = g_new0(Mavlink_Parameter_t, PARAM_COUNT);

        if ((NULL == current_messages) ||
            (NULL == current_parameter))
        {
            g_error("Out of memory!");
        }

        if (NULL == targer_serial_chan)
        {
            // UDP
            GSocket *current_target_socket = g_new0(GSocket, 1);

            if (NULL == current_target_socket)
            {
                g_error("Out of memory!");
            }

            g_message("Found a new system: %d", target_system);
            g_message("Adding new system: %d", target_system);
            as_system_add(target_system, target_autopilot,
                          current_messages,
                          current_parameter,
                          current_target_socket,
                          NULL);
            g_atomic_int_set(vehicle_status + target_system, SYS_DISARMED);
            g_message("New system added: %d", target_system);
        }
        else
        {
            // serial port
            guint8 *current_targer_serial_chan = g_new0(guint8, 1);

            if (NULL == current_targer_serial_chan)
            {
                g_error("Out of memory!");
            }

            g_message("Found a new system: %d", target_system);
            g_message("adding...");
            as_system_add(target_system, target_autopilot,
                          current_messages,
                          current_parameter,
                          NULL,
                          current_targer_serial_chan);
            g_atomic_int_set(vehicle_status + target_system, SYS_DISARMED);
            g_message("New system added: %d", target_system);
        }

        new_system = TRUE; // find new system
    }

    g_mutex_unlock(&message_mutex[target_system]);

    return new_system; // always return at the last of the Func.
}

/**
 * @brief send mavlink message
 * 
 * @param target_system 
 * @param message 
 */
void send_mavlink_message(guint8 target_system, mavlink_message_t *message)
{
    gsize msg_len;
    gchar msg_buf[MAX_BYTES];
    GError *error = NULL;

    static GMutex my_mutex;
    static gint64 last_monotonic_time; //

    // only one write thread
    g_mutex_lock(&my_mutex);

    // make sure msg write interval more than MIN_MSG_INTERVAL
    // TODO: serial port maybe dont need to do this???
    while (g_get_monotonic_time() - last_monotonic_time < MIN_MSG_INTERVAL)
    {
        // g_get_monotonic_time() could return negative value
        // this will fix it.
        if (g_get_monotonic_time() - last_monotonic_time < 0)
        {
            g_usleep(MIN_MSG_INTERVAL);
            break;
        }

        g_usleep(100);
    }
    last_monotonic_time = g_get_monotonic_time();

    gpointer my_system_key = g_atomic_pointer_get(sys_key + target_system);

    g_assert(NULL != my_system_key);
    g_assert(NULL != message);

    g_rw_lock_reader_lock(&target_hash_table_lock);
    gpointer target = g_hash_table_lookup(target_hash_table,
                                          my_system_key);
    g_rw_lock_reader_unlock(&target_hash_table_lock);

    g_assert(NULL != target);

    // Translate message to buffer
    msg_len = mavlink_msg_to_send_buffer((uint8_t *)msg_buf, message);

    // Send
    if (NULL != subnet_address)
    {
        // for UDP "target" is target socket
        g_socket_send((GSocket *)target, msg_buf, msg_len, NULL, &error);

        /* don't forget to check for errors */
        if (error != NULL)
        {
            g_error(error->message);
        }
    }
#ifndef NO_SERISL
    else
    {
        // for serial port "target" is target serial chan
        serial_write_buf_queue_push(*(guint8 *)target, msg_buf, msg_len);
    }
#endif

    g_mutex_unlock(&my_mutex);
}

/**
 * @brief send_heartbeat
 * 
 * @param target_system 
 */
void send_heartbeat(guint8 target_system)
{
    mavlink_message_t message;
    mavlink_heartbeat_t hb;

    hb.type = MAV_TYPE_GCS;
    hb.autopilot = MAV_AUTOPILOT_INVALID;
    hb.base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;
    // hb.mavlink_version = 3; //not writable by user, added by protocol
    mavlink_msg_heartbeat_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &hb);

    send_mavlink_message(target_system, &message);

    // g_print("heart breat sended!\n");
}

/**
 * @brief param_request_list
 * 
 * @param target_system 
 * @param target_autopilot 
 */
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
    send_mavlink_message(target_system, &message);

    // g_message("param_request_list msg wrote!");
}

/**
 * @brief param_request_read
 * 
 * @param target_system 
 * @param target_component 
 * @param param_index 
 */
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

    send_mavlink_message(target_system, &message);
}

/**
 * @brief serial_write_buf_queue_pop
 * 
 * @param chan 
 * @return gchar* 
 */
gchar *serial_write_buf_queue_pop(guint8 chan)
{
    static gchar *last_buf;
    static GMutex my_mutex;

    g_mutex_lock(&my_mutex);
    // only one thread can reach here,
    // avoid repeat free of last_buf

    GAsyncQueue *my_serial_write_buf_queue =
        g_atomic_pointer_get(serial_write_buf_queue + chan);

    if (NULL == my_serial_write_buf_queue)
    {
        return NULL;
    }

    if (NULL != last_buf)
    {
        // free last mavlink_message after pop
        g_free(last_buf);
    }

    last_buf = g_async_queue_try_pop(my_serial_write_buf_queue);

    g_mutex_unlock(&my_mutex);

    return last_buf;
}

/**
 * @brief serial_write_buf_queue_push
 * 
 * @param chan 
 * @param buf 
 * @param buf_len 
 */
#ifndef NO_SERISL
void serial_write_buf_queue_push(guint8 chan, gchar *buf, gsize buf_len)
{
    g_assert(NULL != buf);

    GAsyncQueue *my_serial_write_buf_queue =
        g_atomic_pointer_get(serial_write_buf_queue + chan);

    // serial port not ready
    if (NULL == my_serial_write_buf_queue)
    {
        return;
    }

    if (g_async_queue_length(my_serial_write_buf_queue) >
        MAX_SERIAL_PORT_WRITE_BUF_COUNT)
    {
        g_message("MAX_SERIAL_PORT_WRITE_BUF_COUNT reached!");
        g_message("dump one msg buf!");
        serial_write_buf_queue_pop(chan);
    }

    gchar *serial_write_buf_p = (gchar *)g_new0(gchar, buf_len + 1);

    if (NULL == serial_write_buf_p)
    {
        g_error("Out of memory!");
    }

    serial_write_buf_p[0] = buf_len;

    memcpy(serial_write_buf_p + 1, buf, buf_len);

    g_async_queue_push(my_serial_write_buf_queue,
                       (gpointer)serial_write_buf_p);
}
#endif
