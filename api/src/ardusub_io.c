#include "../inc/ardusub_io.h"
#include "../inc/ardusub_msg.h"

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

void send_heartbeat(guint8 target_system)
{
    mavlink_message_t message;
    mavlink_heartbeat_t hb;

    // TODO: change this value fix value

    hb.type = MAV_TYPE_GCS;
    hb.autopilot = MAV_AUTOPILOT_INVALID;
    hb.base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;
    // hb.mavlink_version = 3; //not writable by user, added by protocol
    mavlink_msg_heartbeat_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &hb);

    send_udp_message(target_system, &message);

    // g_print("heart breat sended!\n");
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
