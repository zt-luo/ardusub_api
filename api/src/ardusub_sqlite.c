/**
 * @file ardusub_sqlite.c
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-02-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#define G_LOG_DOMAIN "[ardusub sqlite    ]"

#include "../inc/ardusub_sqlite.h"
#include "../inc/ardusub_interface.h"

static sqlite3 *sql_db;
static gchar *sql_db_name = "ardusub_api.db";

static gint test_id = 0;

/** sql str definition **/

// usage: "CREATE TABLE " + "`" +"vechle_" + sysid + "`" + sql_str_creat_vechle_table
static gchar *sql_str_creat_vechle_table =
    "( \
    `id`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT, \
    `date`	TEXT, \
    `time`	TEXT, \
    `monotonic_time`	INTEGER, \
    `test_id`	INTEGER, \
    `type`	INTEGER ,\
    `autopilot` INTEGER ,\
    `base_mode` INTEGER,\
    `custom_mode` INTEGER,\
    `system_status` INTEGER,\
    `mavlink_version` INTEGER,\
    `load` INTEGER,\
    `voltage_battery` INTEGER,\
    `current_battery` INTEGER,\
    `drop_rate_comm` INTEGER,\
    `errors_comm` INTEGER,\
    `errors_count1` INTEGER,\
    `errors_count2` INTEGER,\
    `errors_count3` INTEGER,\
    `errors_count4` INTEGER,\
    `battery_remaining` INTEGER,\
    `onboard_control_sensors_present` INTEGER,\
    `onboard_control_sensors_enabled` INTEGER,\
    `onboard_control_sensors_health` INTEGER,\
    `current_consumed` INTEGER,\
    `energy_consumed` INTEGER,\
    `temperature_bs` INTEGER,\
    `current_battery_bs` INTEGER,\
    `battery_id` INTEGER,\
    `battery_function` INTEGER,\
    `type_bs` INTEGER,\
    `battery_remaining_bs` INTEGER,\
    `time_remaining` INTEGER,\
    `charge_state` INTEGER,\
    `Vcc_ps` INTEGER,\
    `Vservo_ps` INTEGER,\
    `flags_ps` INTEGER,\
    `time_unix_usec` INTEGER,\
    `time_boot_ms` INTEGER,\
    `time_boot_ms_at` INTEGER,\
    `roll` REAL,\
    `pitch` REAL,\
    `yaw` REAL,\
    `rollspeed` REAL,\
    `pitchspeed` REAL,\
    `yawspeed` REAL,\
    `time_boot_ms_sp` INTEGER,\
    `press_abs` REAL,\
    `press_diff` REAL,\
    `temperature_sp` INTEGER,\
    `time_boot_ms_sp2` INTEGER,\
    `press_abs2` REAL,\
    `press_diff2` REAL,\
    `temperature2` INTEGER,\
    `time_usec_sor` INTEGER,\
    `servo1_raw` INTEGER,\
    `servo2_raw` INTEGER,\
    `servo3_raw` INTEGER,\
    `servo4_raw` INTEGER,\
    `servo5_raw` INTEGER,\
    `servo6_raw` INTEGER,\
    `servo7_raw` INTEGER,\
    `servo8_raw` INTEGER,\
    `port` INTEGER,\
    `servo9_raw` INTEGER,\
    `servo10_raw` INTEGER,\
    `servo11_raw` INTEGER,\
    `servo12_raw` INTEGER,\
    `servo13_raw` INTEGER,\
    `servo14_raw` INTEGER,\
    `servo15_raw` INTEGER,\
    `servo16_raw` INTEGER,\
    `time_usec_ri` INTEGER,\
    `xacc` INTEGER,\
    `yacc` INTEGER,\
    `zacc` INTEGER,\
    `xgyro` INTEGER,\
    `ygyro` INTEGER,\
    `zgyro` INTEGER,\
    `xmag` INTEGER,\
    `ymag` INTEGER,\
    `zmag` INTEGER,\
    `time_boot_ms_rc` INTEGER,\
    `chan1_raw` INTEGER,\
    `chan2_raw` INTEGER,\
    `chan3_raw` INTEGER,\
    `chan4_raw` INTEGER,\
    `chan5_raw` INTEGER,\
    `chan6_raw` INTEGER,\
    `chan7_raw` INTEGER,\
    `chan8_raw` INTEGER,\
    `chan9_raw` INTEGER,\
    `chan10_raw` INTEGER,\
    `chan11_raw` INTEGER,\
    `chan12_raw` INTEGER,\
    `chan13_raw` INTEGER,\
    `chan14_raw` INTEGER,\
    `chan15_raw` INTEGER,\
    `chan16_raw` INTEGER,\
    `chan17_raw` INTEGER,\
    `chan18_raw` INTEGER,\
    `chancount` INTEGER,\
    `rssi` INTEGER,\
    `time_boot_ms_gpi` INTEGER,\
    `lat` INTEGER,\
    `lon` INTEGER,\
    `alt` INTEGER,\
    `relative_alt` INTEGER,\
    `vx` INTEGER,\
    `vy` INTEGER,\
    `vz` INTEGER,\
    `hdg` INTEGER\
    );";

static gchar *sql_str_insert_vechle_table =
    "INSERT INTO `vehicle_%d` "
    "(date, time, monotonic_time, test_id, type, autopilot, base_mode, custom_mode, system_status, mavlink_version, load, voltage_battery, current_battery, drop_rate_comm, errors_comm, errors_count1, errors_count2, errors_count3, errors_count4, battery_remaining, onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, current_consumed, energy_consumed, temperature_bs, current_battery_bs, battery_id, battery_function, type_bs, battery_remaining_bs, time_remaining, charge_state, Vcc_ps, Vservo_ps, flags_ps, time_unix_usec, time_boot_ms, time_boot_ms_at, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, time_boot_ms_sp, press_abs, press_diff, temperature_sp, time_boot_ms_sp2, press_abs2, press_diff2, temperature2, time_usec_sor, servo1_raw, servo2_raw, servo3_raw, servo4_raw, servo5_raw, servo6_raw, servo7_raw, servo8_raw, port, servo9_raw, servo10_raw, servo11_raw, servo12_raw, servo13_raw, servo14_raw, servo15_raw, servo16_raw, time_usec_ri, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, time_boot_ms_rc, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, chan13_raw, chan14_raw, chan15_raw, chan16_raw, chan17_raw, chan18_raw, chancount, rssi, time_boot_ms_gpi, lat, lon, alt, relative_alt, vx, vy, vz, hdg)"
    "VALUES ('%s', '%s', %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %f, %f, %f, %f, %f, %f, %d, %f, %f, %d, %d, %f, %f, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d);";

/**
 * @brief as_sql_open_db
 * 
 */
void as_sql_open_db()
{
    int rc;
    rc = sqlite3_open(sql_db_name, &sql_db);

    if (SQLITE_OK != rc)
    {
        g_error("Can't open database: %s", sqlite3_errmsg(sql_db));
    }
    else
    {
        g_message("Opened database successfully!");
        as_sql_check_test_info_table();
    }
}

/**
 * @brief as_sql_close_db
 * 
 */
void as_sql_close_db()
{
    gint rc;
    rc = sqlite3_close(sql_db);

    if (SQLITE_OK != rc)
    {
        g_error("%s", sqlite3_errmsg(sql_db));
    }
}

/**
 * @brief as_sql_check_vechle_table
 * 
 * @param sys_id 
 */
void as_sql_check_vechle_table(guint8 sys_id)
{
    // sql statement
    gchar *sql;
    sql = g_new0(gchar, 3000);

    sprintf(sql, "select * from `vehicle_%d`;", sys_id);

    gint rc;
    rc = sqlite3_exec(sql_db, sql, NULL, 0, NULL);

    if (SQLITE_OK != rc)
    {
        sprintf(sql, "CREATE TABLE `vehicle_%d` %s", sys_id, sql_str_creat_vechle_table);
        gchar *errmsg;
        errmsg = g_new0(gchar, 100);
        rc = sqlite3_exec(sql_db, sql, NULL, 0, &errmsg);

        if (SQLITE_OK != rc)
        {
            g_error(errmsg);
        }
        else
        {
            g_message("CREATE TABLE `vehicle_%d`.", sys_id);
        }
        g_free(errmsg);
    }
    else
    {
        g_message("TABLE `vehicle_%d` exist, skip table creat.", sys_id);
    }
    g_free(sql);
}

/**
 * @brief as_sql_insert_vechle_table
 * 
 * @param sys_id 
 * @param vehicle_data 
 */
void as_sql_insert_vechle_table(guint8 sys_id, Vehicle_Data_t *vehicle_data)
{
    // sql statement
    gchar *sql;
    sql = g_new0(gchar, 3000);

    GDateTime *data_time = g_date_time_new_now_local();
    gchar *date_str = g_date_time_format(data_time, "%F");
    gchar *time_str = g_date_time_format(data_time, "%T");

    g_mutex_lock(&vehicle_data_mutex[sys_id]);

    sprintf(sql, sql_str_insert_vechle_table, sys_id,
            date_str,
            time_str,
            g_get_monotonic_time(),
            g_atomic_int_get(&test_id),
            vehicle_data->type,
            vehicle_data->autopilot,
            vehicle_data->base_mode,
            vehicle_data->custom_mode,
            vehicle_data->system_status,
            vehicle_data->mavlink_version,
            vehicle_data->load,
            vehicle_data->voltage_battery,
            vehicle_data->current_battery,
            vehicle_data->drop_rate_comm,
            vehicle_data->errors_comm,
            vehicle_data->errors_count1,
            vehicle_data->errors_count2,
            vehicle_data->errors_count3,
            vehicle_data->errors_count4,
            vehicle_data->battery_remaining,
            vehicle_data->onboard_control_sensors_present,
            vehicle_data->onboard_control_sensors_enabled,
            vehicle_data->onboard_control_sensors_health,
            vehicle_data->current_consumed,
            vehicle_data->energy_consumed,
            vehicle_data->temperature_bs,
            vehicle_data->current_battery_bs,
            vehicle_data->battery_id,
            vehicle_data->battery_function,
            vehicle_data->type_bs,
            vehicle_data->battery_remaining_bs,
            vehicle_data->time_remaining,
            vehicle_data->charge_state,
            vehicle_data->Vcc_ps,
            vehicle_data->Vservo_ps,
            vehicle_data->flags_ps,
            vehicle_data->time_unix_usec,
            vehicle_data->time_boot_ms,
            vehicle_data->time_boot_ms_at,
            vehicle_data->roll,
            vehicle_data->pitch,
            vehicle_data->yaw,
            vehicle_data->rollspeed,
            vehicle_data->pitchspeed,
            vehicle_data->yawspeed,
            vehicle_data->time_boot_ms_sp,
            vehicle_data->press_abs,
            vehicle_data->press_diff,
            vehicle_data->temperature_sp,
            vehicle_data->time_boot_ms_sp2,
            vehicle_data->press_abs2,
            vehicle_data->press_diff2,
            vehicle_data->temperature2,
            vehicle_data->time_usec_sor,
            vehicle_data->servo1_raw,
            vehicle_data->servo2_raw,
            vehicle_data->servo3_raw,
            vehicle_data->servo4_raw,
            vehicle_data->servo5_raw,
            vehicle_data->servo6_raw,
            vehicle_data->servo7_raw,
            vehicle_data->servo8_raw,
            vehicle_data->port,
            vehicle_data->servo9_raw,
            vehicle_data->servo10_raw,
            vehicle_data->servo11_raw,
            vehicle_data->servo12_raw,
            vehicle_data->servo13_raw,
            vehicle_data->servo14_raw,
            vehicle_data->servo15_raw,
            vehicle_data->servo16_raw,
            vehicle_data->time_usec_ri,
            vehicle_data->xacc,
            vehicle_data->yacc,
            vehicle_data->zacc,
            vehicle_data->xgyro,
            vehicle_data->ygyro,
            vehicle_data->zgyro,
            vehicle_data->xmag,
            vehicle_data->ymag,
            vehicle_data->zmag,
            vehicle_data->time_boot_ms_rc,
            vehicle_data->chan1_raw,
            vehicle_data->chan2_raw,
            vehicle_data->chan3_raw,
            vehicle_data->chan4_raw,
            vehicle_data->chan5_raw,
            vehicle_data->chan6_raw,
            vehicle_data->chan7_raw,
            vehicle_data->chan8_raw,
            vehicle_data->chan9_raw,
            vehicle_data->chan10_raw,
            vehicle_data->chan11_raw,
            vehicle_data->chan12_raw,
            vehicle_data->chan13_raw,
            vehicle_data->chan14_raw,
            vehicle_data->chan15_raw,
            vehicle_data->chan16_raw,
            vehicle_data->chan17_raw,
            vehicle_data->chan18_raw,
            vehicle_data->chancount,
            vehicle_data->rssi,
            vehicle_data->time_boot_ms_gpi,
            vehicle_data->lat,
            vehicle_data->lon,
            vehicle_data->alt,
            vehicle_data->relative_alt,
            vehicle_data->vx,
            vehicle_data->vy,
            vehicle_data->vz,
            vehicle_data->hdg);

    g_mutex_unlock(&vehicle_data_mutex[sys_id]);

    g_date_time_unref(data_time);
    g_free(date_str);
    g_free(time_str);

    // g_print(sql);

    gint rc;
    rc = sqlite3_exec(sql_db, sql, NULL, 0, NULL);

    if (SQLITE_OK != rc)
    {
        g_error(sqlite3_errmsg(sql_db));
    }

    g_free(sql);
}

static gchar *sql_str_creat_test_info_table =
    "CREATE TABLE `test_info` \
    (\
    `test_id`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT, \
    `date`	TEXT, \
    `time`	TEXT, \
    `monotonic_time`	INTEGER, \
    `info`	TEXT, \
    `note`	TEXT, \
    `reserved`	TEXT)";

static gchar *sql_str_insert_test_info_table =
    "INSERT INTO `test_info` "
    "(date, time, monotonic_time, info, note, reserved)"
    "VALUES "
    "('%s', '%s', %d, '%s', '%s', '%s');";

static int sql_count_rows_callback(void *unused, int count,
                                   char **data, char **columns);

/**
 * @brief check test_info_table, ino exist creat one.
 * 
 */
void as_sql_check_test_info_table()
{
    // sql statement
    gchar *sql;
    sql = g_new0(gchar, 3000);

    sprintf(sql, "select * from `test_info`;");

    gint rc;
    rc = sqlite3_exec(sql_db, sql, NULL, 0, NULL);

    if (SQLITE_OK != rc)
    {
        gchar *errmsg;
        errmsg = g_new0(gchar, 100);
        rc = sqlite3_exec(sql_db, sql_str_creat_test_info_table, NULL, 0, &errmsg);

        if (SQLITE_OK != rc)
        {
            g_error(errmsg);
        }
        else
        {
            g_message("CREATE TABLE `test_info`.");
        }
        g_free(errmsg);
    }
    else
    {
        g_message("TABLE `test_info` exist, skip table creat.");
    }
    g_free(sql);
}

static gint rows_count = 0;
static const gchar *str_test_info_ptr = "";
static const gchar *str_test_note_ptr = "";

/**
 * @brief insert test_info
 * 
 */
void as_sql_insert_test_info()
{
    // sql statement
    gchar *sql;
    sql = g_new0(gchar, 3000);

    GDateTime *data_time = g_date_time_new_now_local();
    gchar *date_str = g_date_time_format(data_time, "%F");
    gchar *time_str = g_date_time_format(data_time, "%T");

    sprintf(sql, sql_str_insert_test_info_table,
            date_str,
            time_str,
            g_get_monotonic_time(),
            str_test_info_ptr,
            str_test_note_ptr,
            "");

    g_date_time_unref(data_time);
    g_free(date_str);
    g_free(time_str);

    gint rc;
    rc = sqlite3_exec(sql_db, sql, NULL, 0, NULL);

    if (SQLITE_OK != rc)
    {
        g_error(sqlite3_errmsg(sql_db));
    }

    g_free(sql);
}

/**
 * @brief test start
 * 
 * @param test_info must not NULL
 * @param test_note nullable
 */
void as_sql_test_start(const gchar *test_info, const gchar *test_note)
{
    g_assert(NULL != test_info);

    str_test_info_ptr = test_info;
    if (NULL != test_note)
    {
        str_test_note_ptr = test_note;
    }

    as_sql_insert_test_info();

    // sql statement
    gchar *sql;
    sql = g_new0(gchar, 3000);

    sprintf(sql, "select * from `test_info`;");

    gint rc;
    rc = sqlite3_exec(sql_db, sql, sql_count_rows_callback, 0, NULL);

    if (SQLITE_OK != rc)
    {
        gchar *errmsg;
        errmsg = g_new0(gchar, 100);
        g_error(errmsg);
        g_free(errmsg);
    }

    g_free(sql);

    // set test_id
    g_atomic_int_set(&test_id, rows_count);

    rows_count = 0;
}

/**
 * @brief test stop
 * 
 */
void as_sql_test_stop()
{
    g_atomic_int_set(&test_id, 0);
}

/**
 * @brief sqlite exec callback
 * 
 * @param unused 
 * @param count 
 * @param data 
 * @param columns 
 * @return int 
 */
static int sql_count_rows_callback(void *unused, int count,
                                   char **data, char **columns)
{
    g_assert(NULL == unused);
    g_assert(0 != count);
    g_assert(NULL != data);
    g_assert(NULL != columns);

    rows_count++;

    return 0;
}

static gchar *sql_str_creat_command_table =
    "CREATE TABLE `as_command` \
    (\
    `id`	                INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT, \
    `target_system`         INTEGER, \
    `test_id`	            INTEGER, \
    `date`	                TEXT, \
    `time`	                TEXT, \
    `monotonic_time`	    INTEGER, \
    `depth_hold_cmd`	    INTEGER, \
    `depth_hold_depth`	    REAL, \
    `attitude_hold_cmd`	    INTEGER, \
    `attitude_hold_yaw`	    REAL, \
    `attitude_hold_pitch`	REAL, \
    `attitude_hold_roll`	REAL, \
    `flip_trick_type`	    INTEGER, \
    `flip_trick_value`	    REAL)";

static gchar *sql_str_insert_command_table =
    "INSERT INTO `as_command` "
    "(target_system, test_id, date, time, monotonic_time, depth_hold_cmd, depth_hold_depth, attitude_hold_cmd, attitude_hold_yaw, attitude_hold_pitch, attitude_hold_roll, flip_trick_type, flip_trick_value)"
    "VALUES "
    "('%d', '%d', '%s', '%s', %d, '%d', '%f', '%d','%f','%f','%f', '%d', '%f');";

void as_sql_check_command_table()
{
    // sql statement
    gchar *sql;
    sql = g_new0(gchar, 300);

    sprintf(sql, "select * from `as_command`;");

    gint rc;
    rc = sqlite3_exec(sql_db, sql, NULL, 0, NULL);

    if (SQLITE_OK != rc)
    {
        gchar *errmsg;
        errmsg = g_new0(gchar, 100);
        rc = sqlite3_exec(sql_db, sql_str_creat_command_table, NULL, 0, &errmsg);

        if (SQLITE_OK != rc)
        {
            g_error(errmsg);
        }
        else
        {
            g_message("CREATE TABLE `as_command`.");
        }
        g_free(errmsg);
    }
    else
    {
        // g_message("TABLE `as_command` exist, skip table creat.");
    }
    g_free(sql);
}

void as_sql_insert_command(as_command_t as_command)
{
    // sql statement
    gchar *sql;
    sql = g_new0(gchar, 3000);

    GDateTime *data_time = g_date_time_new_now_local();
    gchar *date_str = g_date_time_format(data_time, "%F");
    gchar *time_str = g_date_time_format(data_time, "%T");

    sprintf(sql, sql_str_insert_command_table,
            as_command.target_system,
            g_atomic_int_get(&test_id),
            date_str,
            time_str,
            g_get_monotonic_time(),
            as_command.depth_hold_cmd,
            as_command.depth_hold_depth,
            as_command.attitude_hold_cmd,
            as_command.attitude_hold_yaw,
            as_command.attitude_hold_pitch,
            as_command.attitude_hold_roll,
            as_command.flip_trick_type,
            as_command.flip_trick_value
            );

    g_date_time_unref(data_time);
    g_free(date_str);
    g_free(time_str);

    gint rc;
    rc = sqlite3_exec(sql_db, sql, NULL, 0, NULL);

    if (SQLITE_OK != rc)
    {
        g_error(sqlite3_errmsg(sql_db));
    }

    g_free(sql);
}
