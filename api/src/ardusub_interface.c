/**
 * @file ardusub_interface.c
 * @author ztluo (me@ztluo.dev)
 * @brief interface
 * @version 
 * @date 2019-02-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#define G_LOG_DOMAIN "[ardusub interface ]"

#include "../inc/ardusub_interface.h"

/**
 * @brief init api before use.
 * 
 * @param p_subnet_address ["serial port" for serial port]
 */
void as_api_init(const char *p_subnet_address, const unsigned int flag)
{
    static GMutex my_mutex;
    g_mutex_lock(&my_mutex);

    //! only init once
    if (TRUE != as_init_status)
    {
        // initialize

        for (size_t i = 0; i < 255; i++)
        {
            vehicle_mode[i] = MANUAL;
        }

        thread_flag = flag;

        if (NULL == p_subnet_address)
        {
            subnet_address = SUBNET_ADDRESS;
        }
        else if (0 == g_strcmp0(p_subnet_address, "serial port"))
        {
            subnet_address = NULL;

            #ifdef NO_SERISL
            g_error("NO serial port support on Windows!");
            #endif
        }
        else
        {
            subnet_address = g_strdup(p_subnet_address);
        }

        as_thread_init_ptr_flag();

        message_hash_table = g_hash_table_new(g_int_hash, g_int_equal);
        parameter_hash_table = g_hash_table_new(g_int_hash, g_int_equal);
        manual_control_table = g_hash_table_new(g_int_hash, g_int_equal);
        target_hash_table = g_hash_table_new(g_int_hash, g_int_equal);

        // load ini config file
        if (thread_flag & F_STORAGE_INI)
        {
            as_read_ini_file();
        }

        // set log handler
        if (thread_flag & F_STORAGE_LOG)
        {
            as_set_log_handler();
        }

        // init database
        if (thread_flag & F_STORAGE_DATABASE)
        {
            as_sql_open_db();
        }

        if (NULL != subnet_address)
        {
            // UDP here
            as_udp_read_init();
        }
        else
        {
            // serial port here
#ifndef NO_SERISL
            as_serial_read_init();
#else
            g_error("NO serial port support on Windows!");
#endif
        }

        as_api_main_thread =
            g_thread_new("as_api_main", &as_run, NULL);

        as_init_status = TRUE;
    }

    g_mutex_unlock(&my_mutex);
}

/**
 * @brief deinit api.
 * 
 */
void as_api_deinit()
{
    gint wait_count = 0;
    while (TRUE != as_init_status)
    {
        g_usleep(100000);
        wait_count++;
        if (wait_count++ > 100)
        {
            return;
        }
    }

    as_thread_stop_all_join();
}

/**
 * @brief api main loop GLib thread.
 * 
 * @param data 
 * @return gpointer 
 */
gpointer as_run(gpointer data)
{
    g_assert(NULL == data);

    as_main_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(as_main_loop);
    // g_main_loop_unref(as_main_loop);

    return NULL;
}

/**
 * @brief add system.
 * 
 * @param target_system 
 * @param target_autopilot 
 * @param current_messages 
 * @param current_parameter 
 * @param current_target_socket 
 * @param current_targer_serial_chan 
 */
void as_system_add(guint8 target_system, guint8 target_autopilot,
                   Mavlink_Messages_t *current_messages,
                   Mavlink_Parameter_t *current_parameter,
                   GSocket *current_target_socket,
                   guint8 *current_targer_serial_chan)
{
    g_assert(current_messages != NULL);
    g_assert(current_parameter != NULL);
    g_assert((current_target_socket != NULL) ||
             (current_targer_serial_chan != NULL));

    g_atomic_int_inc(&sys_count);

    guint8 *p_sysid = g_new0(guint8, 1);
    if (NULL == p_sysid)
    {
        g_error("Out of memory!");
    }
    *p_sysid = target_system;

    // lock the hash table
    g_rw_lock_writer_lock(&message_hash_table_lock);
    g_rw_lock_writer_lock(&parameter_hash_table_lock);
    g_rw_lock_writer_lock(&target_hash_table_lock);

    g_hash_table_insert(message_hash_table, p_sysid, current_messages);

    g_hash_table_insert(parameter_hash_table, p_sysid, current_parameter);

    if (NULL != current_target_socket)
    {
        // UDP
        as_udp_write_init(target_system, current_target_socket);
        g_hash_table_insert(target_hash_table, p_sysid, current_target_socket);
    }

#ifndef NO_SERISL
    if (NULL != current_targer_serial_chan)
    {
        // serial port
        as_serial_write_init();
        g_hash_table_insert(target_hash_table, p_sysid, current_targer_serial_chan);
    }
#endif

    mavlink_manual_control_t *p_manual_control = g_new0(mavlink_manual_control_t, 1);
    p_manual_control->z = 500; // 500 is z axis zero leval
    g_hash_table_insert(manual_control_table, p_sysid, p_manual_control);

    // unlock the message hash table
    g_rw_lock_writer_unlock(&message_hash_table_lock);
    g_rw_lock_writer_unlock(&parameter_hash_table_lock);
    g_rw_lock_writer_unlock(&target_hash_table_lock);

    statustex_queue[target_system] = g_async_queue_new();
    named_val_float_queue[target_system] = g_async_queue_new();
    message_queue[target_system] = g_async_queue_new();

    Vehicle_Data_t *p_vehicle_data = g_new0(Vehicle_Data_t, 1);
    if (NULL == p_vehicle_data)
    {
        g_error("Out of memory!");
    }
    g_atomic_pointer_set(vehicle_data_array + target_system, p_vehicle_data);

    g_atomic_pointer_set(sys_key + target_system, p_sysid);

    heartbeat_thread[target_system] =
        g_thread_new("heartbeat_worker", &heartbeat_worker, p_sysid);

    if (thread_flag & F_THREAD_FETCH_FULL_PARAM)
    {
        as_request_full_parameters(target_system, target_autopilot);
    }

    as_reauest_data_stream(target_system, target_autopilot);

    // init manual_control_worker thread
    manual_control_thread[target_system] =
        g_thread_new("manual_control_worker", &manual_control_worker, p_sysid);

    // init named_val_float_handle_worker thread
    if (thread_flag & F_THREAD_NAMED_VAL_FLOAT)
    {
        named_val_float_handle_thread[target_system] =
            g_thread_new("named_val_float_handle_worker", &named_val_float_handle_worker, p_sysid);
    }

    // init vehicle_data_update_worker thread
    vehicle_data_update_thread[target_system] =
        g_thread_new("vehicle_data_update_worker", &vehicle_data_update_worker, p_sysid);

    // init db_update_worker thread
    if (thread_flag & F_STORAGE_DATABASE)
    {
        db_update_thread[target_system] =
            g_thread_new("db_update_worker", &db_update_worker, p_sysid);
    }

    // statustex_wall_thread
    if (thread_flag & F_THREAD_STATUSTEX_WALL)
    {
        statustex_wall_thread[target_system] =
            g_thread_new("statustex_wall_worker", &statustex_wall_worker, p_sysid);
    }
}

/**
 * @brief set manual control value.
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param r 
 * @param buttons 
 * @param ... sys_id for multiple system 
 */
void as_api_manual_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, ...)
{
    uint8_t sys_id = 1;

    va_list ap;
    va_start(ap, 1);
    sys_id = va_arg(ap, int);
    va_end(ap);

    if (0 == as_api_check_vehicle(sys_id))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  sys_id, __FILE__, __FUNCTION__, __LINE__);
    }

    // arm first
    if (SYS_ARMED != g_atomic_int_get(vehicle_status + sys_id))
    {
        return;
    }

    g_rw_lock_reader_lock(&manual_control_hash_table_lock);
    mavlink_manual_control_t *p_manual_control =
        g_hash_table_lookup(manual_control_table,
                            g_atomic_pointer_get(sys_key + sys_id));
    g_rw_lock_reader_unlock(&manual_control_hash_table_lock);

    g_mutex_lock(&manual_control_mutex[sys_id]); // lock
    p_manual_control->target = sys_id;
    p_manual_control->x = x;
    p_manual_control->y = y;
    p_manual_control->z = z;
    p_manual_control->r = r;
    p_manual_control->buttons = buttons;
    g_mutex_unlock(&manual_control_mutex[sys_id]); // unlock
}

/**
 * @brief get vehicles data.
 * 
 * @param target_system 
 * @return Vehicle_Data_t* 
 */
Vehicle_Data_t *as_api_get_vehicle_data(uint8_t target_system)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);
        return NULL;
    }

    static Vehicle_Data_t *last_vehicle_data;
    static GMutex my_mutex;

    g_mutex_lock(&my_mutex);
    // only one thread can reach here

    if (NULL != last_vehicle_data)
    {
        g_free(last_vehicle_data);
    }

    g_mutex_lock(&vehicle_data_mutex[target_system]);
    last_vehicle_data = g_memdup(
        g_atomic_pointer_get(vehicle_data_array + target_system),
        sizeof(Vehicle_Data_t));
    g_mutex_unlock(&vehicle_data_mutex[target_system]);

    g_mutex_unlock(&my_mutex);

    last_vehicle_data->monotonic_time = g_get_monotonic_time();

    return last_vehicle_data;
}

/**
 * @brief get vehicles data 2.
 * 
 * @param target_system 
 * @return int 1 for success
 */
int as_api_get_vehicle_data2(uint8_t target_system, Vehicle_Data_t *vehicle_data)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);
        return 0;
    }

    g_mutex_lock(&vehicle_data_mutex[target_system]);
    memcpy(
        (void *)vehicle_data,
        (void *)g_atomic_pointer_get(vehicle_data_array + target_system),
        sizeof(Vehicle_Data_t));
    g_mutex_unlock(&vehicle_data_mutex[target_system]);

    vehicle_data->monotonic_time = g_get_monotonic_time();

    return 1;
}

/**
 * @brief get message
 * 
 * @param sysid 
 * @return Mavlink_Messages_t* 
 */
Mavlink_Messages_t *as_get_message(uint8_t sysid)
{
    gpointer system_key_ = g_atomic_pointer_get(sys_key + sysid);
    g_assert(NULL != system_key_);

    g_rw_lock_reader_lock(&message_hash_table_lock);
    Mavlink_Messages_t *p_message = g_hash_table_lookup(message_hash_table, system_key_);
    g_rw_lock_reader_unlock(&message_hash_table_lock);

    g_assert(NULL != p_message);

    return p_message;
}

/**
 * @brief check if vehicle is ready.
 * 
 * @param sysid 
 * @return int 0 for not ready.
 */
int as_api_check_vehicle(uint8_t sysid)
{
    if ((NULL == g_atomic_pointer_get(sys_key + sysid)) ||
        (SYS_UN_INIT == g_atomic_int_get(vehicle_status + sysid)) ||
        (SYS_INITIATING == g_atomic_int_get(vehicle_status + sysid)))
    {
        return 0;
    }
    else
    {
        return !0;
    }
}

/**
 * @brief 
 * 
 * @param as_api_set_servo 
 * @param target_autopilot 
 * @param servo_no 
 * @param pwm 
 */
void as_api_set_servo(uint8_t target_system,
                      uint8_t target_autopilot,
                      float servo_no, float pwm)
{

    mavlink_command_long_t cmd_long;
    cmd_long.target_system = target_system;
    cmd_long.target_component = target_autopilot;
    cmd_long.command = MAV_CMD_DO_SET_SERVO;
    cmd_long.confirmation = 0;
    cmd_long.param1 = servo_no;
    cmd_long.param2 = pwm;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &cmd_long);

    send_mavlink_message(target_system, &message);
}

/**
 * @brief as_api_motor_test
 * 
 * @param target_system 
 * @param target_autopilot 
 * @param motor_no 
 * @param pwm 
 */
void as_api_motor_test(uint8_t target_system,
                       uint8_t target_autopilot,
                       float motor_no, float pwm)
{

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

    mavlink_message_t message;
    mavlink_msg_command_long_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &cmd_long);

    send_mavlink_message(target_system, &message);
}

/**
 * @brief set vehicle mode
 * 
 * @param target_system 
 * @param mode 
 */
void as_api_set_mode(uint8_t target_system, control_mode_t mode)
{
    // TODO: need check
    mavlink_set_mode_t set_mode;
    set_mode.target_system = target_system;
    set_mode.base_mode = 209; //81
    set_mode.custom_mode = mode;

    mavlink_message_t message;
    mavlink_msg_set_mode_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &set_mode);

    send_mavlink_message(target_system, &message);

    printf("sys_id: %d, set mode to %d \n", target_system, mode);

    g_atomic_int_set(vehicle_mode + target_system, mode);
}

/**
 * @brief request full parameters
 * 
 * @param target_system 
 * @param target_component 
 */
void as_request_full_parameters(guint8 target_system, guint8 target_component)
{
    guint16 *target_ = NULL;
    target_ = g_new0(guint16, 1);
    g_assert(NULL != target_);
    *target_ = target_system << 8;
    *target_ |= target_component;

    parameters_request_thread =
        g_thread_new("parameters_request_worker", &parameters_request_worker, target_);
}

/**
 * @brief 
 * 
 * @param target_system 
 * @param target_autopilot 
 * @param ch1 
 * @param ch2 
 * @param ch3 
 * @param ch4 
 * @param ch5 
 * @param ch6 
 * @param ch7 
 * @param ch8 
 */
void as_api_send_rc_channels_override(uint8_t target_system, uint8_t target_autopilot,
                                      uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4,
                                      uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8)
{
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

    mavlink_message_t message;
    mavlink_msg_rc_channels_override_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message,
                                            &rc_channels_override);

    send_mavlink_message(target_system, &message);
}

/**
 * @brief send NAMED_VALUE_FLOAT ( #251 )
 * 
 * @param name 
 * @param value 
 */
void as_api_send_named_value_float(uint8_t target_system, char *name, float value)
{
    mavlink_named_value_float_t nvf;
    nvf.time_boot_ms = 0;
    nvf.value = value;
    strcpy(nvf.name, name);

    mavlink_message_t message;
    mavlink_msg_named_value_float_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message,
                                         &nvf);
    send_mavlink_message(target_system, &message);
}

/**
 * @brief send NAMED_VALUE_INT ( #252 )
 * 
 * @param name 
 * @param value 
 */
void as_api_send_named_value_int(uint8_t target_system, char *name, int value)
{
    mavlink_named_value_int_t nvi;
    nvi.time_boot_ms = 0;
    nvi.value = value;
    strcpy(nvi.name, name);

    mavlink_message_t message;
    mavlink_msg_named_value_int_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message,
                                       &nvi);
    send_mavlink_message(target_system, &message);
}

/**
 * @brief send REQUEST_DATA_STREAM ( #66 )
 * 
 * @param target_system 
 * @param target_component 
 * @param req_stream_id 
 * @param req_message_rate 
 * @param start_stop 
 */
void as_send_request_data_stream(guint8 target_system, guint8 target_component,
                                 guint8 req_stream_id, guint16 req_message_rate,
                                 guint8 start_stop)
{
    mavlink_request_data_stream_t rds;
    rds.target_system = target_system;
    rds.target_component = target_component;
    rds.req_stream_id = req_stream_id;
    rds.req_message_rate = req_message_rate;
    rds.start_stop = start_stop;

    mavlink_message_t message;
    mavlink_msg_request_data_stream_encode(STATION_SYSYEM_ID,
                                           STATION_COMPONENT_ID,
                                           &message, &rds);

    send_mavlink_message(target_system, &message);
}

/**
 * @brief as_reauest_data_stream
 * 
 * @param target_system 
 * @param target_component 
 */
void as_reauest_data_stream(guint8 target_system, guint8 target_component)
{
    guint16 *target_ = NULL;
    target_ = g_new0(guint16, 1);
    g_assert(NULL != target_);
    *target_ = target_system << 8;
    *target_ |= target_component;

    request_data_stream_thread =
        g_thread_new("request_data_stream_worker", &request_data_stream_worker, target_);
}

/**
 * @brief vehicle arm
 * 
 * @param target_system 
 * @param target_autopilot 
 */
void as_api_vehicle_arm(uint8_t target_system, uint8_t target_autopilot)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);
    }

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
        g_hash_table_lookup(manual_control_table,
                            g_atomic_pointer_get(sys_key + target_system));
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
    send_mavlink_message(target_system, &message);
}

/**
 * @brief vehicle disarm
 * 
 * @param target_system 
 * @param target_autopilot 
 */
void as_api_vehicle_disarm(uint8_t target_system, uint8_t target_autopilot)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);
    }

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
    send_mavlink_message(target_system, &message);

    g_atomic_int_set(vehicle_status + target_system, SYS_DISARMED);

    g_rw_lock_reader_lock(&manual_control_hash_table_lock);
    mavlink_manual_control_t *p_manual_control =
        g_hash_table_lookup(manual_control_table,
                            g_atomic_pointer_get(sys_key + target_system));
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

/**
 * @brief set desire depth
 * 
 * @param target_system 
 * @param cmd 
 * @param depth 
 */
void as_api_depth_hold(uint8_t target_system, uint8_t cmd, float depth)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);
    }

    mavlink_depth_hold_t dh = {0};

    dh.cmd = cmd;
    dh.depth = depth;

    mavlink_message_t message;
    mavlink_msg_depth_hold_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &dh);

    send_mavlink_message(target_system, &message);

    as_command_t as_command = {0};
    as_command.target_system = target_system;
    as_command.depth_hold_cmd = cmd;
    as_command.depth_hold_depth = depth;

    as_insert_command(as_command);
}

/**
 * @brief set desire attitude
 * 
 * @param target_system 
 * @param cmd 
 * @param yaw 
 * @param pitch 
 * @param roll 
 */
void as_api_attitude_hold(uint8_t target_system, uint8_t cmd, float yaw, float pitch, float roll)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);
    }

    mavlink_attitude_hold_t ah = {0};

    ah.cmd = cmd;
    ah.yaw = yaw;
    ah.pitch = pitch;
    ah.roll = roll;

    mavlink_message_t message;
    mavlink_msg_attitude_hold_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &ah);

    send_mavlink_message(target_system, &message);

    as_command_t as_command = {0};
    as_command.target_system = target_system;
    as_command.attitude_hold_cmd = cmd;
    as_command.attitude_hold_yaw = yaw;
    as_command.attitude_hold_pitch = pitch;
    as_command.attitude_hold_roll = roll;

    as_insert_command(as_command);
}

/**
 * @brief do flip trick 
 * 
 * @param target_system 
 * @param type 
 * @param value 
 */
void as_api_flip_trick(uint8_t target_system, uint8_t type, float value)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);
    }

    mavlink_flip_trick_t ft = {0};

    ft.tpye = type;
    ft.value = value;

    mavlink_message_t message;
    mavlink_msg_flip_trick_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID, &message, &ft);

    send_mavlink_message(target_system, &message);

    as_command_t as_command = {0};
    as_command.target_system = target_system;
    as_command.flip_trick_type = type;
    as_command.flip_trick_value = value;

    as_insert_command(as_command);
}

void as_api_depth_pid(uint8_t target_system, uint8_t save, float kp,
                      float ki, float kd, float imax, float filt_hz, float ff)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);
    }

    // mavlink_lab_depth_pid_t ld_pid = {save, p, i, d, imax, filt_hz, ff};

    mavlink_message_t message;
    // mavlink_msg_lab_depth_pid_encode(STATION_SYSYEM_ID, STATION_COMPONENT_ID,
    //                                  &message, &ld_pid);
    mavlink_msg_lab_depth_pid_pack(STATION_SYSYEM_ID, STATION_COMPONENT_ID,
                                   &message, save, kp, ki, kd, imax, filt_hz, ff);

    send_mavlink_message(target_system, &message);
}

/**
 * @brief pop statustex.
 * 
 * ! NULL-able return value
 * 
 * @param target_system 
 * @return mavlink_statustext_t* 
 */
mavlink_statustext_t *as_api_statustex_queue_pop(uint8_t target_system)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);

        return NULL;
    }

    return statustex_queue_pop(target_system);
}

/**
 * @brief statustex count
 * 
 * @param target_system 
 * @return int 
 */
int as_api_statustex_count(uint8_t target_system)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);

        return 0;
    }

    return g_async_queue_length(statustex_queue[target_system]);
}

/**
 * @brief pop statustex 
 * 
 * @param target_system 
 * @return mavlink_statustext_t* 
 */
mavlink_statustext_t *statustex_queue_pop(guint8 target_system)
{
    static mavlink_statustext_t *last_statustex;
    static GMutex my_mutex;

    g_mutex_lock(&my_mutex);
    // only one thread can reach here,
    // avoid repeat free of last_statustex

    GAsyncQueue *my_statustex_queue =
        g_atomic_pointer_get(statustex_queue + target_system);

    if (NULL == my_statustex_queue)
    {
        return NULL;
    }

    if (NULL != last_statustex)
    {
        // free last statustex after pop
        g_free(last_statustex);
    }

    last_statustex = g_async_queue_try_pop(my_statustex_queue);

    g_mutex_unlock(&my_mutex);

    return last_statustex;
}

/**
 * @brief push statustex 
 * 
 * @param target_system 
 * @param current_messages 
 */
void statustex_queue_push(guint8 target_system,
                          Mavlink_Messages_t *current_messages)
{
    g_assert(NULL != current_messages);

    GAsyncQueue *my_statustex_queue =
        g_atomic_pointer_get(statustex_queue + target_system);

    if (NULL == my_statustex_queue)
    {
        return;
    }

    if (g_async_queue_length(my_statustex_queue) > MAX_STATUSTEX)
    {
        g_critical("MAX_STATUSTEX reached!");
        statustex_queue_pop(target_system);
    }

    gpointer statustex_p = g_memdup(&current_messages->statustext,
                                    sizeof(mavlink_statustext_t));

    if (NULL == statustex_p)
    {
        g_error("Out of memory!");
    }

    g_async_queue_push(my_statustex_queue, // queue
                       statustex_p);
}

mavlink_named_value_float_t *as_api_named_val_float_queue_pop(guint8 target_system)
{
    if (0 == as_api_check_vehicle(target_system))
    {
        g_warning("no vehicle id:%d, in file: %s, func: %s, line: %d",
                  target_system, __FILE__, __FUNCTION__, __LINE__);

        return NULL;
    }

    return named_val_float_queue_pop(target_system);
}

/**
 * @brief pop named_val_float 
 * 
 * @param target_system 
 * @return mavlink_named_value_float_t* 
 */
mavlink_named_value_float_t *named_val_float_queue_pop(guint8 target_system)
{
    static mavlink_named_value_float_t *last_named_val_float;
    static GMutex my_mutex;

    g_mutex_lock(&my_mutex);
    // only one thread can reach here,
    // avoid repeat free of last_named_val_float

    GAsyncQueue *my_named_val_float_queue =
        g_atomic_pointer_get(named_val_float_queue + target_system);

    if (NULL == my_named_val_float_queue)
    {
        return NULL;
    }

    if (NULL != last_named_val_float)
    {
        // free last named_val_float after pop
        g_free(last_named_val_float);
    }

    last_named_val_float = g_async_queue_try_pop(my_named_val_float_queue);

    g_mutex_unlock(&my_mutex);

    return last_named_val_float;
}

/**
 * @brief named_val_float push 
 * 
 * @param target_system 
 * @param current_messages 
 */
void named_val_float_queue_push(guint8 target_system,
                                Mavlink_Messages_t *current_messages)
{
    g_assert(NULL != current_messages);

    GAsyncQueue *my_named_val_float_queue =
        g_atomic_pointer_get(named_val_float_queue + target_system);

    if (NULL == my_named_val_float_queue)
    {
        return;
    }

    if (g_async_queue_length(my_named_val_float_queue) > MAX_NAMED_VALUE_FLOAT)
    {
        g_critical("MAX_NAMED_VALUE_FLOAT reached!");
        named_val_float_queue_pop(target_system);
    }

    gpointer named_val_float_p = g_memdup(&current_messages->named_value_float,
                                          sizeof(mavlink_named_value_float_t));

    if (NULL == named_val_float_p)
    {
        g_error("Out of memory!");
    }

    g_async_queue_push(my_named_val_float_queue, // queue
                       named_val_float_p);
}

/**
 * @brief message pop 
 * 
 * @param target_system 
 * @return Mavlink_Messages_t* 
 */
Mavlink_Messages_t *message_queue_pop(guint8 target_system)
{
    static Mavlink_Messages_t *last_mavlink_message;
    static GMutex my_mutex;

    g_mutex_lock(&my_mutex);
    // only one thread can reach here,
    // avoid repeat free of last_mavlink_message

    GAsyncQueue *my_message_queue =
        g_atomic_pointer_get(message_queue + target_system);

    if (NULL == my_message_queue)
    {
        return NULL;
    }

    if (NULL != last_mavlink_message)
    {
        // free last mavlink_message after pop
        g_free(last_mavlink_message);
    }

    last_mavlink_message = g_async_queue_try_pop(my_message_queue);

    g_mutex_unlock(&my_mutex);

    return last_mavlink_message;
}

/**
 * @brief message push 
 * 
 * @param target_system 
 * @param current_messages 
 */
void message_queue_push(guint8 target_system,
                        Mavlink_Messages_t *current_messages)
{
    g_assert(NULL != current_messages);

    GAsyncQueue *my_message_queue =
        g_atomic_pointer_get(message_queue + target_system);

    if (NULL == my_message_queue)
    {
        return;
    }

    if (g_async_queue_length(my_message_queue) > MAX_MESSAGE)
    {
        g_critical("MAX_MESSAGE reached!");
        message_queue_pop(target_system);
    }

    gpointer mavlink_message_p = g_memdup(current_messages,
                                          sizeof(Mavlink_Messages_t));

    if (NULL == mavlink_message_p)
    {
        g_error("Out of memory!");
    }

    g_async_queue_push(my_message_queue, // queue
                       mavlink_message_p);
}

/**
 * @brief as_sql_test_start wrapper 
 * 
 * @param test_info 
 * @param test_note 
 */
void as_api_test_start(const char *test_info, const char *test_note)
{
    as_sql_test_start(test_info, test_note);
}

/**
 * @brief as_sql_test_stop wrapper 
 * 
 */
void as_api_test_stop()
{
    as_sql_test_stop();
}

void as_insert_command(as_command_t as_command)
{
    // as_command_p free at db_insert_command_worker
    as_command_t *as_command_p =
        (as_command_t *)g_memdup(&as_command, sizeof(as_command_t));

    g_thread_new("", db_insert_command_worker, (gpointer)as_command_p);
}