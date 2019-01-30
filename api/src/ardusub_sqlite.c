#include "ardusub_sqlite.h"

/** sql str definition **/
static gchar *sql_creat_table = "CREATE TABLE ";

// usage: sql_creat_table + "vechle_" + sysid + sql_vechle_table
static gchar *sql_vechle_table = "( \
    `id`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT, \
    `time`	TEXT NOT NULL, \
    `monotonic time`	INTEGER, \
    `type`	INTEGER \
    );";
