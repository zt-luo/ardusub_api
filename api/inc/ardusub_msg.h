#include "ardusub_def.h"
#include "ardusub_interface.h"

guint8 as_handle_messages(gchar *msg_tmp, gsize bytes_read);
void as_handle_message_id(mavlink_message_t message,
                          Mavlink_Messages_t *current_messages,
                          Mavlink_Parameter_t *current_parameter);
