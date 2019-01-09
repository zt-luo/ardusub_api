#include <stdlib.h>

#include <glib.h>
#include <glib-object.h>
#include <json-glib/json-glib.h>
#include <common/mavlink.h>

gsize raw_msg_count = 0;
gsize decoded_msg_count = 0;

static gboolean raw_hex = FALSE;
static gchar *json_file = NULL;

static mavlink_message_t message = {0};
static mavlink_status_t status = {0};

static GOptionEntry entries[] =
    {
        {"json", 'j', 0, G_OPTION_ARG_FILENAME, &json_file, "Decode from json file", "<path to MAVLink msg json file>"},
        {"raw", 'r', 0, G_OPTION_ARG_NONE, &raw_hex, "Decode from raw hex", "<raw hex separated by spaces>"},
        {NULL}};

gsize cal_str_len(gchar *str);
void as_handle_message_id(mavlink_message_t message);

void decode_from_json_file();
void decode_from_raw_hex(int argc, char const *argv[]);

void json_iterator(JsonArray *array, guint index_,
                   JsonNode *element_node, gpointer user_data);

int main(int argc, char const *argv[])
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
    g_option_group_free(group);

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
        return EXIT_FAILURE;
    }
    else
    {
        g_print("%s opend!\n",  json_file);
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

    g_print("Got %d raw msg data from json file: %s ...\n",
            raw_msg_count, json_file);
    g_print("----------------------------------------------\n\n");

    json_array_foreach_element(json_node_get_array(root), json_iterator, NULL);

    g_object_unref(parser);
    g_object_unref(reader);

    g_print("------------------------------------------------\n");
    g_print("%d msg decoded from json file: %s.\n",
            decoded_msg_count, json_file);
}

void decode_from_raw_hex(int argc, char const *argv[])
{
    raw_msg_count = argc - 1;

    g_print("Got %d raw msg data from raw hex data...\n", raw_msg_count);
    g_print("------------------------------------------------\n");

    for (int i = 2; i < argc; i++)
    {
        g_print("%s", argv[i]);
        gsize str_len = cal_str_len(argv[i]);
        gchar str_char[3] = {0};
        guint8 num = 0;

        for (int j = 0; j < str_len; j++)
        {
            str_char[0] = argv[i][j];
            j++;
            str_char[1] = argv[i][j];
            guint8 num = (guint8)strtol(str_char, NULL, 16);

            // g_print("%02x", num);
            gboolean msg_received = mavlink_parse_char(MAVLINK_COMM_1,
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
        }
    }

    g_print("------------------------------------------------\n");
    g_print("%d msg decoded from raw hex data.\n",
            decoded_msg_count);
}

gsize cal_str_len(gchar *str)
{
    gsize len = 0;
    gint i = 0;
    gchar c = str[i];

    while (c != NULL)
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
    // g_print("this is element %d\n", index_);

    JsonReader *reader = json_reader_new(element_node);
    json_reader_read_member(reader, "_source");
    json_reader_read_member(reader, "layers");
    json_reader_read_member(reader, "data");

    json_reader_read_member(reader, "data.data");
    gchar *data_str;
    data_str = json_reader_get_string_value(reader);
    // g_print("data:%s\n", data_str);
    json_reader_end_member(reader);

    if (TRUE != json_reader_read_member(reader, "data.len"))
        g_error("Broken json file.\n");
    gchar *data_len_str;
    data_len_str = json_reader_get_string_value(reader);
    gsize data_len = (gsize)strtol(data_len_str, NULL, 10);
    // g_print("data_len:%d\n", data_len);

    gchar **data_str_split;
    data_str_split = g_strsplit(data_str, ":", -1);
    gchar *data_member = {0};

    for (gsize i = 0; i < data_len; i++)
    {
        data_member = data_str_split[i];

        g_print("%s", data_member);

        guint8 num = (guint8)strtol(data_member, NULL, 16);
        gboolean msg_received = mavlink_parse_char(MAVLINK_COMM_1,
                                                   num, &message,
                                                   &status);

        if (TRUE == msg_received)
        {
            decoded_msg_count++;
            // g_print("msg id: %d\n", message.msgid);
            as_handle_message_id(message);
            g_print("\n");
        }
    }

    g_strfreev(data_str_split);

    g_object_unref(reader);
}

void as_handle_message_id(mavlink_message_t message)
{
    g_print(" FROM sysid:%d, compid:%d\n",
            message.sysid, message.compid);

    // Handle Message ID
    switch (message.msgid)
    {

        /* MAY THE SOURCE BE WITH YOU */

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&message, &hb);

        g_print("HEARTBEAT(#0) -> ");
        g_print("type:%d, autopilot:%d, base_mode:%d, custom_mode:%d, system_status:%d, mavlink_version:%d \n",
                hb.type, hb.autopilot, hb.base_mode, hb.custom_mode,
                hb.system_status, hb.mavlink_version);

        break;
    }

    case MAVLINK_MSG_ID_SYSTEM_TIME:
    {
        mavlink_system_time_t st;

        mavlink_msg_system_time_decode(&message, &st);

        g_print("SYSTEM_TIME(#2) -> ");
        g_print("time_unix_usec:%d, time_boot_ms:%d \n",
                st.time_unix_usec, st.time_boot_ms);

        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        mavlink_param_request_read_t prr;

        mavlink_msg_param_request_read_decode(&message, &prr);

        g_print("PARAM_REQUEST_READ(#20) -> ");
        g_print("target_system:%d, target_component:%d, param_id:%d, param_index:%d \n",
                prr.target_system, prr.target_component,
                prr.param_id, prr.param_index);

        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        mavlink_param_request_list_t prl;

        mavlink_msg_param_request_list_decode(&message, &prl);

        g_print("PARAM_REQUEST_LIST(#21) -> ");
        g_print("target_system:%d, target_component:%d \n",
                prl.target_system, prl.target_system);

        break;
    }

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        mavlink_mission_request_list_t mrl;

        mavlink_msg_mission_request_list_decode(&message, &mrl);

        g_print("MISSION_REQUEST_LIST(#43) -> ");
        g_print("target_system:%d, target_component:%d, mission_type:%d \n",
                mrl.target_system, mrl.target_component, mrl.mission_type);

        break;
    }

    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        mavlink_mission_ack_t ma;

        mavlink_msg_mission_ack_decode(&message, &ma);

        g_print("MISSION_ACK(#47) -> ");
        g_print("target_system:%d, target_component:%d, type:%d, mission_type:%d \n",
                ma.target_system, ma.target_component, ma.type, ma.mission_type);

        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        mavlink_request_data_stream_t rds;

        mavlink_msg_request_data_stream_decode(&message, &rds);

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

        mavlink_msg_manual_control_decode(&message, &mc);

        gchar buttons_str[17] = {0};
        itoa(mc.buttons, buttons_str, 2);

        g_print("MANUAL_CONTROL(#69) -> ");
        g_print("target:%d, x:%d, y:%d, z:%d, r:%d, buttons:%s \n",
                mc.target, mc.x, mc.y, mc.z, mc.r, buttons_str);

        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        mavlink_command_long_t cl;

        mavlink_msg_command_long_decode(&message, &cl);

        g_print("COMMAND_LONG (#76) -> ");
        g_print("target_system:%d, target_component:%d, command:%d, confirmation:%d, ",
                cl.target_system, cl.target_component, cl.command, cl.confirmation);
        g_print("param1:%d, param2:%d, param3:%d, param4:%d, param5:%d, param6:%d, param7:%d \n",
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
