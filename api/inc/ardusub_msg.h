/**
 * @file ardusub_msg.h
 * @author ztluo (me@ztluo.dev)
 * @brief 
 * @version 
 * @date 2019-02-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include "ardusub_def.h"
#include "ardusub_interface.h"

guint8 as_handle_messages(mavlink_message_t message);
void as_handle_message_id(mavlink_message_t message,
                          Mavlink_Messages_t *current_messages,
                          Mavlink_Parameter_t *current_parameter);
void as_handle_named_value_float(guint8 target_system,
                                Mavlink_Messages_t *current_messages);
