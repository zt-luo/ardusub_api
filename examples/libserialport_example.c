/**
 * @file libserialport_example.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 0.1
 * @date 2019-01-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <stdio.h>

#include <glib.h>
#include <glib/gstdio.h>
#include <libserialport.h>

gsize count_serial_port(struct sp_port **list_ptr);
gpointer port_read_worker(gpointer data);

/**
 * @brief main func
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char const *argv[])
{
    g_assert(0 != argc);
    g_assert(NULL != argv);

    struct sp_port **port_list;
    struct sp_port *port_pix = NULL;
    enum sp_return sp_result;

    sp_result = sp_list_ports(&port_list);

    if (SP_OK == sp_result)
    {
        g_print("Enumerating the serial ports...\n");

        gsize sp_count = 0;
        gint vid = 0, pid = 0;
        sp_count = count_serial_port(port_list);
        g_print("ok. got %lu ports.\n", sp_count);

        for (gsize i = 0; i < sp_count; i++)
        {
            vid = 0;
            pid = 0;
            sp_result = sp_get_port_usb_vid_pid(port_list[i], &vid, &pid);

            g_print("port%lu description:%s, vid:%d, pid:%d. ",
                    i,
                    sp_get_port_description(port_list[i]),
                    vid,
                    pid);
            if (vid == 9900 && pid == 17)
            {
                g_print("Pixhawk device\n");
                port_pix = port_list[i];
            }
            else
            {
                g_print("\n");
            }
        }
    }

    // port_pix=port_list[2];

    if (NULL != port_pix)
    {
        // struct sp_port_config *sp_config;
        // sp_new_config(&sp_config);
        // sp_result = sp_get_config(port_pix, sp_config);
        // if (SP_OK != sp_result)
        // {
        //     g_error("config faild!");
        // }
        if (SP_OK != sp_open(port_pix, SP_MODE_READ_WRITE))
        {
            g_error("port open faild!");
        }

        g_print("port opened.\n");

        g_thread_new("read thread", port_read_worker, port_pix);

        while (TRUE)
        {
            g_usleep(1000);
        }
    }

    sp_free_port_list(port_list);

    return 0;
}

/**
 * @brief count the serial port according to the list_ptr 
 * given by sp_list_ports.
 * 
 * @param list_ptr 
 * @return gsize 
 */
gsize count_serial_port(struct sp_port **list_ptr)
{
    gsize serial_count = 0;
    while (NULL != list_ptr[serial_count])
    {
        serial_count++;
    }

    return serial_count;
}

/**
 * @brief read one byte and print it 
 * 
 * @param data 
 * @return gpointer 
 */
gpointer port_read_worker(gpointer data)
{
    guint8 buf;
    enum sp_return sp_result;

    while (TRUE)
    {
        sp_result = sp_blocking_read((struct sp_port *)data, &buf, 1, 0);
        sp_result = sp_blocking_write((struct sp_port *)data, &buf, 1, 0);

        if (SP_OK!=sp_result)
        {
            g_error("error in serial port read or write!");
        }

        g_print("%02x", buf);
    }
}
