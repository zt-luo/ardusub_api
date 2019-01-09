#include <stdlib.h>

#include <glib.h>
#include <glib-object.h>
#include <json-glib/json-glib.h>
#include <common/mavlink.h>

gsize cal_str_len(gchar *str);
void as_handle_message_id(mavlink_message_t message);
void json_iterator(JsonArray *array, guint index_, JsonNode *element_node, gpointer user_data);

mavlink_message_t message;
mavlink_status_t status;

gsize raw_msg_count = 0;
gsize decoded_msg_count = 0;

int main(int argc, char const *argv[])
{
    JsonParser *parser;
    JsonNode *root, *udp_pack, *data;
    JsonReader *reader;
    GError *error = NULL;

    parser = json_parser_new();

    json_parser_load_from_file(parser, "123.json", &error);

    if (error)
    {
        g_print("Unable to parse `%s': %s\n", argv[1], error->message);
        g_error_free(error);
        g_object_unref(parser);
        return EXIT_FAILURE;
    }
    else
    {
        g_print("123.json opend!\n");
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

    g_print("Got %d raw msg data...\n", raw_msg_count);
    g_print("-------------------------------\n\n");

    json_array_foreach_element(json_node_get_array(root), json_iterator, NULL);

    g_object_unref(parser);

    for (int i = 1; i < argc; i++)
    {
        g_print("%s\n", argv[i]);
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
            gboolean msg_received = mavlink_parse_char(MAVLINK_COMM_1, num, &message, &status);

            if (TRUE == msg_received)
            {
                decoded_msg_count++;
                // g_print("msg id: %d\n", message.msgid);
                as_handle_message_id(message);
                g_print("\n");
            }
        }
    }

    g_print("-------------------------------\n");
    g_print("%d msg decoded.\n", decoded_msg_count);

    return 0;
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

void json_iterator(JsonArray *array, guint index_, JsonNode *element_node, gpointer user_data)
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

        if (data_len - 1 == i)
        {
            g_print("\n");
        }

        guint8 num = (guint8)strtol(data_member, NULL, 16);
        gboolean msg_received = mavlink_parse_char(MAVLINK_COMM_1, num, &message, &status);

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
    // Handle Message ID
    switch (message.msgid)
    {
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        mavlink_manual_control_t mc;

        mavlink_msg_manual_control_decode(&message, &mc);

        gchar buttons_str[17] = {0};
        itoa(mc.buttons, buttons_str, 2);

        g_print("MANUAL_CONTROL(#69) -> ");
        g_print("x:%d, y:%d, z:%d, r:%d, buttons:%s\n", mc.x, mc.y, mc.z, mc.r, buttons_str);

        break;
    }

        // case MAVLINK_MSG_ID_HEARTBEAT:
        // {
        //     // printf("MAVLINK_MSG_ID_HEARTBEAT\n");
        //     mavlink_msg_heartbeat_decode(&message, &(current_messages->heartbeat));

        //     /* Queries the system monotonic time. in microseconds (gint64) */
        //     // g_get_monotonic_time();
        //     /* https://developer.gnome.org/glib/stable/glib-Date-and-Time-Functions.html#g-get-monotonic-time */

        //     current_messages->time_stamps.heartbeat = g_get_monotonic_time();

        //     /* MAY THE SOURCE BE WITH YOU */
        //     // printf("timestamp:%d\n", current_messages->time_stamps.heartbeat);
        //     // printf("Received message from sys:%d|comp:%d\n",
        //     //        current_messages->sysid, current_messages->compid);
        //     // printf("type:%d, autopilot:%d, base_mode:%d, custom_mode:%d, system_status:%d, mavlink_version:%d \n",
        //     //        current_messages->heartbeat.type, current_messages->heartbeat.autopilot,
        //     //        current_messages->heartbeat.base_mode, current_messages->heartbeat.custom_mode,
        //     //        current_messages->heartbeat.system_status, current_messages->heartbeat.mavlink_version);

        //     g_message("heartbeat msg from system:%d", current_messages->sysid);

        //     // send heartbeat
        //     send_heartbeat();

        //     break;
        // }

        // case MAVLINK_MSG_ID_SYS_STATUS:
        // {
        //     // printf("MAVLINK_MSG_ID_SYS_STATUS\n");
        //     mavlink_msg_sys_status_decode(&message, &(current_messages->sys_status));
        //     // printf("SYS_STATUS: erros_comm %d %d \n", current_messages->sys_status.voltage_battery,
        //     //    current_messages->sys_status.current_battery);
        //     current_messages->time_stamps.sys_status = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_PING:
        // {
        //     printf("MAVLINK_MSG_ID_PING\n");
        //     mavlink_msg_ping_decode(&message, &(current_messages->ping));
        //     printf("PING: time_usec:%d, seq:%d, target_system:%d, target_component:%d",
        //            current_messages->ping.time_usec, current_messages->ping.seq,
        //            current_messages->ping.target_system, current_messages->ping.target_component);

        //     break;
        // }

        // case MAVLINK_MSG_ID_BATTERY_STATUS:
        // {
        //     // printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
        //     mavlink_msg_battery_status_decode(&message, &(current_messages->battery_status));
        //     current_messages->time_stamps.battery_status = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_RADIO_STATUS:
        // {
        //     // printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
        //     mavlink_msg_radio_status_decode(&message, &(current_messages->radio_status));
        //     current_messages->time_stamps.radio_status = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        // {
        //     //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
        //     mavlink_msg_local_position_ned_decode(&message, &(current_messages->local_position_ned));
        //     current_messages->time_stamps.local_position_ned = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        // {
        //     //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
        //     mavlink_msg_global_position_int_decode(&message, &(current_messages->global_position_int));
        //     current_messages->time_stamps.global_position_int = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        // {
        //     //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
        //     mavlink_msg_position_target_local_ned_decode(&message, &(current_messages->position_target_local_ned));
        //     current_messages->time_stamps.position_target_local_ned = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
        // {
        //     // printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
        //     mavlink_msg_position_target_global_int_decode(&message, &(current_messages->position_target_global_int));
        //     current_messages->time_stamps.position_target_global_int = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_HIGHRES_IMU:
        // {
        //     // printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
        //     mavlink_msg_highres_imu_decode(&message, &(current_messages->highres_imu));
        //     current_messages->time_stamps.highres_imu = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_ATTITUDE:
        // {
        //     // printf("MAVLINK_MSG_ID_ATTITUDE\n");
        //     mavlink_msg_attitude_decode(&message, &(current_messages->attitude));
        //     current_messages->time_stamps.attitude = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        // {
        //     //printf("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW\n");
        //     mavlink_msg_servo_output_raw_decode(&message, &(current_messages->servo_output_raw));
        //     current_messages->time_stamps.servo_output_raw = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_COMMAND_ACK:
        // {
        //     // printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
        //     mavlink_msg_command_ack_decode(&message, &(current_messages->command_ack));
        //     current_messages->time_stamps.command_ack = g_get_monotonic_time();
        //     // printf("Command_ACK, command:%d, result:%d. \n", current_messages->command_ack.command,
        //     //        current_messages->command_ack.result);
        //     break;
        // }

        // case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        // {
        //     // printf("MAVLINK_MSG_ID_NAMED_VALUE_FLOA\n");
        //     mavlink_msg_named_value_float_decode(&message, &(current_messages->named_value_float));
        //     current_messages->time_stamps.named_value_float = g_get_monotonic_time();
        //     // printf(current_messages->named_value_float.name);
        //     // printf(": %f \n", current_messages->named_value_float.value);
        //     break;
        // }

        // case MAVLINK_MSG_ID_VFR_HUD:
        // {
        //     // printf("MAVLINK_MSG_ID_VFR_HUD\n");
        //     mavlink_msg_vfr_hud_decode(&message, &(current_messages->vfr_hud));
        //     current_messages->time_stamps.vfr_hud = g_get_monotonic_time();
        //     // printf("heading:%d\n", current_messages->vfr_hud.heading);
        //     break;
        // }

        // case MAVLINK_MSG_ID_POWER_STATUS:
        // {
        //     // printf("MAVLINK_MSG_ID_POWER_STATUS\n");
        //     mavlink_msg_power_status_decode(&message, &(current_messages->power_status));
        //     current_messages->time_stamps.power_status = g_get_monotonic_time();
        //     // printf("Vcc(5V rail voltage in mV):%d, Vservo(servo rail voltage in mV):%d, "
        //     // "power supply status flags:%d.\n", current_messages->power_status.Vcc,
        //     // current_messages->power_status.Vservo, current_messages->power_status.flags);
        //     break;
        // }

        // case MAVLINK_MSG_ID_SYSTEM_TIME:
        // {
        //     // printf("MAVLINK_MSG_ID_SYSTEM_TIME\n");
        //     mavlink_msg_system_time_decode(&message, &(current_messages->system_time));
        //     current_messages->time_stamps.system_time = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_MISSION_CURRENT:
        // {
        //     // printf("MAVLINK_MSG_ID_MISSION_CURRENT\n");
        //     mavlink_msg_mission_current_decode(&message, &(current_messages->mission_current));
        //     current_messages->time_stamps.mission_current = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_GPS_RAW_INT:
        // {
        //     // printf("MAVLINK_MSG_ID_GPS_RAW_INT\n");
        //     mavlink_msg_gps_raw_int_decode(&message, &(current_messages->gps_raw_int));
        //     current_messages->time_stamps.gps_raw_int = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
        // {
        //     // printf("MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT\n");
        //     mavlink_msg_nav_controller_output_decode(&message, &(current_messages->nav_controller_output));
        //     current_messages->time_stamps.nav_controller_output = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_RC_CHANNELS:
        // {
        //     // printf("MAVLINK_MSG_ID_RC_CHANNELS\n");
        //     mavlink_msg_rc_channels_decode(&message, &(current_messages->rc_channels));
        //     current_messages->time_stamps.rc_channels = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_VIBRATION:
        // {
        //     // printf("MAVLINK_MSG_ID_VIBRATION\n");
        //     mavlink_msg_vibration_decode(&message, &(current_messages->vibration));
        //     current_messages->time_stamps.vibration = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_RAW_IMU:
        // {
        //     // printf("MAVLINK_MSG_ID_RAW_IMU\n");
        //     mavlink_msg_raw_imu_decode(&message, &(current_messages->raw_imu));
        //     current_messages->time_stamps.raw_imu = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_SCALED_PRESSURE:
        // {
        //     // printf("MAVLINK_MSG_ID_SCALED_PRESSURE\n");
        //     mavlink_msg_scaled_pressure_decode(&message, &(current_messages->scaled_pressure));
        //     current_messages->time_stamps.scaled_pressure = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_SCALED_IMU2:
        // {
        //     // printf("MAVLINK_MSG_ID_SCALED_IMU2\n");
        //     mavlink_msg_scaled_imu2_decode(&message, &(current_messages->scaled_imu2));
        //     current_messages->time_stamps.scaled_imu2 = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_SCALED_PRESSURE2:
        // {
        //     // printf("MAVLINK_MSG_ID_SCALED_PRESSURE2\n");
        //     mavlink_msg_scaled_pressure2_decode(&message, &(current_messages->scaled_pressure2));
        //     current_messages->time_stamps.scaled_pressure2 = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        // {
        //     // printf("MAVLINK_MSG_ID_RC_CHANNELS_RAW\n");
        //     mavlink_msg_rc_channels_raw_decode(&message, &(current_messages->rc_channels_raw));
        //     current_messages->time_stamps.rc_channels_raw = g_get_monotonic_time();
        //     break;
        // }

        // case MAVLINK_MSG_ID_STATUSTEXT:
        // {
        //     // printf("MAVLINK_MSG_ID_STATUSTEXT\n");
        //     mavlink_msg_statustext_decode(&message, &(current_messages->statustext));
        //     current_messages->time_stamps.statustext = g_get_monotonic_time();
        //     printf("severity: %d, statustext: ", current_messages->statustext.severity);
        //     printf(current_messages->statustext.text);
        //     printf("\n");
        //     break;
        // }

        // case MAVLINK_MSG_ID_PARAM_VALUE:
        // {
        //     // printf("MAVLINK_MSG_ID_PARAM_VALUE\n");
        //     mavlink_msg_param_value_decode(&message, &(current_messages->param_value));
        //     current_messages->time_stamps.param_value = g_get_monotonic_time();

        //     guint16 _param_index = current_messages->param_value.param_index;

        //     if (_param_index > PARAM_COUNT - 1)
        //     {
        //         g_warning("param_index out of range! param_index:%d, PARAM_COUNT:%d\n", _param_index, PARAM_COUNT);
        //     }
        //     else
        //     {
        //         for (gint i = 0; i < 16; i++)
        //         {
        //             current_parameter[_param_index].param_id[i] =
        //                 current_messages->param_value.param_id[i];
        //         }

        //         current_parameter[_param_index].param_type =
        //             current_messages->param_value.param_type;

        //         current_parameter[_param_index].param_value.param_float =
        //             current_messages->param_value.param_value;
        //     }

        //     // printf("param_id:%s, param_value:%d, param_type:%d, param_count:%d, param_index:%d\n",
        //     //        current_messages->param_value.param_id, current_messages->param_value.param_value,
        //     //        current_messages->param_value.param_type, current_messages->param_value.param_count,
        //     //        current_messages->param_value.param_index);

        //     break;
        // }

    default:
    {
        g_message("Warning, did not handle message id %i\n", message.msgid);
        break;
    }

    } // end: switch msgid
}