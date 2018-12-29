/**
 * @file main.c
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
#include <gio/gio.h>

#include <common/mavlink.h>

#include "ardusub_interface.h"

/**
 * @brief 
 * 
 * @param parameter_size 
 * @param parameter 
 * @return int 
 */
int main(int argc, char *argv[])
{
    // gchar ch[128];
    // g_sprintf(ch, "hello world");
    // g_printf(g_strup(ch));
    // g_print("\n");
    // g_printf(g_strdown(ch));
    // g_print("\n");

    // gint i = 0;
    // gchar chch[2] = {0x00};
    // chch[0] = ch[i];

    // while(0x00 != chch[0])
    // {
    //     g_printf(chch);
    //     i++;
    //     chch[0] = ch[i];
    // }

    // g_print("\n");
    // g_print("\n");

    // mavlink_heartbeat_t hb;
    // hb.type = 12;
    // hb.autopilot = 3;
    // hb.base_mode = 209;
    // hb.custom_mode = 0;
    // hb.system_status = 3;
    // hb.mavlink_version = 3;
    // mavlink_msg_heartbeat_encode(255, 0, &message, &hb);
    // send(sock, (char *)&message, (int)strlen((char *)&message), 0);

    // gint erro_code;
    // if (0 != erro_code)
    // {
    //     g_printf("bind erro code: %d \n", erro_code);
    //     return -1;
    // }

    // while (0 == msgReceived)
    // {
    //     // g_io_channel_read_chars(p_udp_socket, (gchar *)&cp, 1, &bytes_read, &udp_read_error);
    //     recv(sock, (char *)&cp, 1, 0);
    //     g_printf("%d", cp);
    //     msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
    // }

    //

    // GSocket *socket_udp_read;
    // GError *error = NULL;

    // GSocketAddress *gsock_addr_read = G_SOCKET_ADDRESS(
    //     // g_inet_socket_address_new(g_inet_address_new_from_string("192.168.1.120"), 14551));
    //     g_inet_socket_address_new(g_inet_address_new_any(G_SOCKET_FAMILY_IPV4), 14551));
    // socket_udp_read = g_socket_new(G_SOCKET_FAMILY_IPV4, G_SOCKET_TYPE_DATAGRAM, G_SOCKET_PROTOCOL_UDP, &error);

    // GSocketAddress *gsock_addr_write = G_SOCKET_ADDRESS(
    //     g_inet_socket_address_new(g_inet_address_new_from_string("192.168.1.2"), 14551));
    // socket_udp_write = g_socket_new(G_SOCKET_FAMILY_IPV4, G_SOCKET_TYPE_DATAGRAM, G_SOCKET_PROTOCOL_UDP, &error);

    // if (FALSE == g_socket_connect(socket_udp_write, gsock_addr_write, NULL, &error))
    // {
    //     g_error(error->message);
    // }

    // if (socket_udp_read == NULL)
    // {
    //     g_print("ERROR NEW socket\n");
    //     g_error(error->message);
    // }

    // if (g_socket_bind(socket_udp_read, gsock_addr_read, TRUE, &error) == FALSE)
    // {
    //     g_print("ERROR bind\n");
    //     g_error(error->message);
    // }

    // gint fd = g_socket_get_fd(socket_udp_read);

    // // GIOChannel *channel = g_io_channel_unix_new(fd);
    // GIOChannel *channel = g_io_channel_win32_new_socket(fd);
    // g_io_channel_set_encoding(channel, NULL, &error);
    // guint source = g_io_add_watch(channel, G_IO_IN, (GIOFunc)read_udp_socket, socket_udp_write);

    ardusub_api_init();

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);
    g_main_loop_unref(loop);

    return 0;
}
