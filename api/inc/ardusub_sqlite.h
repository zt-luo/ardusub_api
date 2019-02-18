#include "ardusub_def.h"

#include "../../sqlite/sqlite3.h"

void as_sql_open_db();
void as_sql_close_db();
void as_sql_check_vechle_table(guint8 sys_id);
void as_sql_insert_vechle_table(guint8 sys_id, Vehicle_Data_t *vehicle_data);
