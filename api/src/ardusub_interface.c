#include "../inc/ardusub_interface.h"

void as_api_init(char *p_subnet_address)
{
    static gboolean as_init_status;

    static GMutex my_mutex;
    g_mutex_lock(&my_mutex);

    //! only init once
    if (TRUE != as_init_status)
    {
        // initialize

        if (NULL == p_subnet_address)
        {
            subnet_address = SUBNET_ADDRESS;
        }
        else if (0 == g_strcmp0(p_subnet_address, "serial port"))
        {
            subnet_address = NULL;
        }
        else
        {
            subnet_address = p_subnet_address;
        }

        message_hash_table = g_hash_table_new(g_int_hash, g_int_equal);
        parameter_hash_table = g_hash_table_new(g_int_hash, g_int_equal);
        manual_control_table = g_hash_table_new(g_int_hash, g_int_equal);
        target_hash_table = g_hash_table_new(g_int_hash, g_int_equal);

        if (NULL != subnet_address)
        {
            // UDP here
            as_udp_read_init();
        }
        else
        {
            // serial port here
            as_serial_read_init();
        }

        g_thread_new("as_api_main", &as_run, NULL);

        // sqlite3 *my_db;
        // sqlite3_open("test.db", &my_db);

        as_init_status = TRUE;
    }

    g_mutex_unlock(&my_mutex);
}

void as_api_deinit()
{
    ;
}

gpointer as_run(gpointer data)
{
    g_assert(NULL == data);

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);
    g_main_loop_unref(loop);

    return NULL;
}

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

    if (NULL != current_targer_serial_chan)
    {
        // serial port
        as_serial_write_init();
        g_hash_table_insert(target_hash_table, p_sysid, current_targer_serial_chan);
    }

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

    as_request_full_parameters(target_system, target_autopilot);

    // init manual_control_worker thread
    g_thread_new("manual_control_worker", &manual_control_worker, p_sysid);

    // init named_val_float_handle_worker thread
    g_thread_new("named_val_float_handle_worker", &named_val_float_handle_worker, p_sysid);

    // init vehicle_data_update_worker thread
    g_thread_new("vehicle_data_update_worker", &vehicle_data_update_worker, p_sysid);
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
        g_hash_table_lookup(manual_control_table,
                            g_atomic_pointer_get(sys_key + sys_id));
    g_rw_lock_reader_unlock(&manual_control_hash_table_lock);

    g_mutex_lock(&manual_control_mutex[sys_id]); // lock
    p_manual_control->x = x;
    p_manual_control->y = y;
    p_manual_control->z = z;
    p_manual_control->r = r;
    p_manual_control->buttons = buttons;
    g_mutex_unlock(&manual_control_mutex[sys_id]); // unlock
}

Vehicle_Data_t *as_api_get_vehicle_data(uint8_t target_system)
{
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
        sizeof(Mavlink_Messages_t));
    g_mutex_unlock(&vehicle_data_mutex[target_system]);

    g_mutex_unlock(&my_mutex);

    return last_vehicle_data;
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
    send_mavlink_message(target_system, &message);

    // g_print("do_set_servo msg wrote!");
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
    g_print("do_motor_test msg wrote!");
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
    g_print("do_set_mode msg wrote!");
    // check the write
}

void as_request_full_parameters(guint8 target_system, guint8 target_component)
{
    guint16 *target_ = NULL;
    target_ = g_new0(guint16, 1);
    g_assert(NULL != target_);
    *target_ = target_system << 8;
    *target_ |= target_component;

    g_thread_new("parameters_request_worker", &parameters_request_worker, target_);
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
    g_print("send_rc_channels_override msg wrote!");
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
        named_val_float_queue_pop(target_system);
    }

    gpointer named_val_float_p = g_memdup(&current_messages->named_value_float,
                                          sizeof(mavlink_statustext_t));

    if (NULL == named_val_float_p)
    {
        g_error("Out of memory!");
    }

    g_async_queue_push(my_named_val_float_queue, // queue
                       named_val_float_p);
}

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
        g_message("MAX_MESSAGE reached!");
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
