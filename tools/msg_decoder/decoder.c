/**
 * @file decoder.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 0.1
 * @date 2019-01-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/* MAVlink */
// max MAVlink channels
#define MAVLINK_COMM_NUM_BUFFERS (1)

#include <stdlib.h>

#include <glib.h>
#include <glib-object.h>
#include <json-glib/json-glib.h>
#include <common/mavlink.h>

guint32 raw_msg_count = 0;
guint32 decoded_msg_count = 0;

static gboolean raw_hex = FALSE;
static gchar *json_file = NULL;

static mavlink_message_t message = {0};
static mavlink_status_t status = {0};

static GOptionEntry entries[] =
    {
        {"json", 'j', 0, G_OPTION_ARG_FILENAME, &json_file, "Decode from json file", "<path to MAVLink msg json file>"},
        {"raw", 'r', 0, G_OPTION_ARG_NONE, &raw_hex, "Decode from raw hex", "<raw hex separated by spaces>"},
        {NULL}};

gsize cal_str_len(const gchar *str);
void as_handle_message_id(mavlink_message_t message);

void decode_from_json_file();
void decode_from_raw_hex(int argc, char *argv[]);

void json_iterator(JsonArray *array, guint index_,
                   JsonNode *element_node, gpointer user_data);

int main(int argc, char *argv[])
{
    GError *error = NULL;

    GOptionContext *context;
    GOptionGroup *group;
    gchar **args;

#ifdef G_OS_WIN32
    args = g_win32_get_command_line();
#else
    args = g_strdupv(argv);
#endif

    // set up context
    context = g_option_context_new(
        "- decode MAVLink msg frome json file or raw hex data");
    g_option_context_add_main_entries(context, entries, NULL);
    group = g_option_group_new("Application", "description",
                               "help_description", NULL, NULL);
    g_option_context_add_group(context, group);

    if (!g_option_context_parse_strv(context, &args, &error))
    {
        // error happened
        g_error(error->message);
    }

    g_option_context_set_help_enabled(context, TRUE);

    // free something
    g_strfreev(args);
    g_option_context_free(context);
    g_option_group_unref(group);

    if (NULL != json_file)
    {
        decode_from_json_file();
    }
    else if (TRUE == raw_hex)
    {
        decode_from_raw_hex(argc, argv);
    }
    else
    {
        g_message("no valid data input.");
    }

    g_free(json_file);

    g_print("\a");
    g_message("Done.");

    return 0;
}

void decode_from_json_file()
{
    GError *error = NULL;

    JsonParser *parser;
    JsonNode *root;
    JsonReader *reader;

    parser = json_parser_new();

    json_parser_load_from_file(parser, json_file, &error);

    if (error)
    {
        g_print("Unable to parse json file %s : %s\n",
                json_file, error->message);
        g_error_free(error);
        g_object_unref(parser);
        return;
    }
    else
    {
        g_print("%s opend!\n", json_file);
    }

    root = json_parser_get_root(parser);
    json_node_seal(root);
    reader = json_reader_new(root);

    if (FALSE == json_reader_is_array(reader))
    {
        g_error("Broken json file.\n");
    }

    raw_msg_count = json_reader_count_elements(reader);
    // g_print("count elements %d\n", raw_msg_count);

    g_print("Got %d raw MAVLink message data \nfrom json file: '%s' ...\n",
            raw_msg_count, json_file);
    g_print("------------------------------------------------------\n\n");

    json_array_foreach_element(json_node_get_array(root), json_iterator, NULL);

    g_print("------------------------------------------------------\n");
    g_print("%d MAVLink message decoded \nfrom json file: '%s'.\n",
            decoded_msg_count, json_file);
}

void decode_from_raw_hex(int argc, char *argv[])
{
    raw_msg_count = argc - 2;

    g_print("Got %d raw MAVLink message data \nfrom raw hex data...\n", raw_msg_count);
    g_print("------------------------------------------------------\n");

    for (int i = 2; i < argc; i++)
    {
        g_print("[%d]%s\n", i - 1, argv[i]);
        gsize str_len = cal_str_len(argv[i]);
        gchar str_char[3] = {0};

        if (0 != str_len % 2)
        {
            g_print("BROKEN_MESSAGE(#-1) -> !!\n\n");
        }

        for (guint8 j = 0; j < str_len; j++)
        {
            str_char[0] = argv[i][j];
            j++;
            str_char[1] = argv[i][j];
            guint8 num = (guint8)strtol(str_char, NULL, 16);

            // g_print("%02x", num);
            gboolean msg_received = mavlink_parse_char(MAVLINK_COMM_0,
                                                       num,
                                                       &message,
                                                       &status);

            if (TRUE == msg_received)
            {
                decoded_msg_count++;
                // g_print("msg id: %d\n", message.msgid);
                as_handle_message_id(message);
                g_print("\n");
            }
            else if (j == str_len - 1)
            {
                g_print("BROKEN_MESSAGE(#-1) -> !!\n\n");
            }
        }
    }

    g_print("------------------------------------------------------\n");
    g_print("%d MAVLink message decoded \nfrom raw hex data.\n",
            decoded_msg_count);
}

gsize cal_str_len(const gchar *str)
{
    gsize len = 0;
    gint i = 0;
    gchar c = str[i];

    while (c != 0)
    {
        len++;
        i++;
        c = str[i];

        if (i > 279)
        {
            return 0;
        }
    }

    return len;
}

void json_iterator(JsonArray *array, guint index_,
                   JsonNode *element_node,
                   gpointer user_data)
{

    g_assert(NULL != array);
    g_assert(NULL != element_node);
    g_assert(NULL == user_data);

    // g_print("this is element %d\n", index_);

    JsonReader *reader = json_reader_new(element_node);
    json_reader_read_member(reader, "_source");
    json_reader_read_member(reader, "layers");

    // read frame.time
    json_reader_read_member(reader, "frame");
    json_reader_read_member(reader, "frame.time");
    gchar *time_str;
    const gchar *c_time_str;
    c_time_str = json_reader_get_string_value(reader);
    time_str = g_memdup(c_time_str, 32);
    time_str[31] = 0;
    // reset cursor to "layers"
    json_reader_end_member(reader);
    json_reader_end_member(reader);

    // read data.data and data.len
    json_reader_read_member(reader, "data");
    json_reader_read_member(reader, "data.data");
    const gchar *data_str = json_reader_get_string_value(reader);
    // g_print("data:%s\n", data_str);
    json_reader_end_member(reader);

    if (TRUE != json_reader_read_member(reader, "data.len"))
        g_error("Broken json file.\n");
    const gchar *data_len_str = json_reader_get_string_value(reader);
    gsize data_len = (gsize)strtol(data_len_str, NULL, 10);
    // g_print("data_len:%d\n", data_len);

    gchar **data_str_split;
    data_str_split = g_strsplit(data_str, ":", -1);
    gchar *data_member = {0};

    g_print("[%d]", index_ + 1);

    for (gsize i = 0; i < data_len; i++)
    {
        data_member = data_str_split[i];

        g_print("%s", data_member);

        guint8 num = (guint8)strtol(data_member, NULL, 16);
        gboolean msg_received = mavlink_parse_char(MAVLINK_COMM_0,
                                                   num, &message,
                                                   &status);

        if (TRUE == msg_received)
        {
            decoded_msg_count++;

            g_print("\n%s", time_str);
            g_print(" FROM sysid:%d, compid:%d\n",
                    message.sysid, message.compid);

            as_handle_message_id(message);
            g_print("\n");
        }
        else if (i == data_len - 1)
        {
            g_print("\n%s", time_str);
            g_print(" FROM sysid:NO, compid:NO\n");
            g_print("BROKEN_MESSAGE(#-1) -> !!\n\n");
        }
    }

    g_free((gpointer)c_time_str);
    g_free(time_str);
    g_free((gpointer)data_str);
    g_free((gpointer)data_len_str);

    g_strfreev(data_str_split);

    g_object_unref(reader);
}

void as_handle_message_id(mavlink_message_t msg)
{
    // Handle Message ID
    switch (msg.msgid)
    {

        /* MAY THE SOURCE BE WITH YOU */

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);

        g_print("HEARTBEAT(#0) -> ");
        g_print("type:%d, autopilot:%d, base_mode:%d, custom_mode:%d, system_status:%d, mavlink_version:%d \n",
                hb.type, hb.autopilot, hb.base_mode, hb.custom_mode,
                hb.system_status, hb.mavlink_version);

        break;
    }

    case MAVLINK_MSG_ID_SYSTEM_TIME:
    {
        mavlink_system_time_t st;

        mavlink_msg_system_time_decode(&msg, &st);

        g_print("SYSTEM_TIME(#2) -> ");
        g_print("time_unix_usec:%llu, time_boot_ms:%d \n",
                st.time_unix_usec, st.time_boot_ms);

        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        mavlink_param_request_read_t prr;

        mavlink_msg_param_request_read_decode(&msg, &prr);

        g_print("PARAM_REQUEST_READ(#20) -> ");
        g_print("target_system:%d, target_component:%d, param_id:%s, param_index:%d \n",
                prr.target_system, prr.target_component,
                prr.param_id, prr.param_index);

        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        mavlink_param_request_list_t prl;

        mavlink_msg_param_request_list_decode(&msg, &prl);

        g_print("PARAM_REQUEST_LIST(#21) -> ");
        g_print("target_system:%d, target_component:%d \n",
                prl.target_system, prl.target_system);

        break;
    }

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        mavlink_mission_request_list_t mrl;

        mavlink_msg_mission_request_list_decode(&msg, &mrl);

        g_print("MISSION_REQUEST_LIST(#43) -> ");
        g_print("target_system:%d, target_component:%d, mission_type:%d \n",
                mrl.target_system, mrl.target_component, mrl.mission_type);

        break;
    }

    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        mavlink_mission_ack_t ma;

        mavlink_msg_mission_ack_decode(&msg, &ma);

        g_print("MISSION_ACK(#47) -> ");
        g_print("target_system:%d, target_component:%d, type:%d, mission_type:%d \n",
                ma.target_system, ma.target_component, ma.type, ma.mission_type);

        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        mavlink_request_data_stream_t rds;

        mavlink_msg_request_data_stream_decode(&msg, &rds);

        g_print("REQUEST_DATA_STREAM(#66) -> ");
        g_print("target_system:%d, target_component:%d, req_stream_id:%d, ",
                rds.target_system, rds.target_component, rds.req_stream_id);
        g_print("req_message_rate:%d, start_stop:%d \n",
                rds.req_message_rate, rds.start_stop);
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        mavlink_manual_control_t mc;

        mavlink_msg_manual_control_decode(&msg, &mc);

        g_print("MANUAL_CONTROL(#69) -> ");
        g_print("target:%d, x:%d, y:%d, z:%d, r:%d, buttons:%d \n",
                mc.target, mc.x, mc.y, mc.z, mc.r, mc.buttons);

        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        mavlink_command_long_t cl;

        mavlink_msg_command_long_decode(&msg, &cl);

        g_print("COMMAND_LONG (#76) -> ");
        g_print("target_system:%d, target_component:%d, command:%d, confirmation:%d, ",
                cl.target_system, cl.target_component, cl.command, cl.confirmation);
        g_print("param1:%f, param2:%f, param3:%f, param4:%f, param5:%f, param6:%f, param7:%f \n",
                cl.param1, cl.param2, cl.param3, cl.param4, cl.param5, cl.param6, cl.param7);

        break;
    }

    default:
    {
        g_message("Warning, did not handle message id %i\n", message.msgid);
        break;
    }
    }
}
