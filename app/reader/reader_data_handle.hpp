#pragma once

#include "ubeacon_driver_common.h"
#include "ubeacon_driver_data.h"

namespace data_handle {
bool on_frame_begin(void *arg, const uint8_t *uid, ub_frame_id_t frame_id);
void on_frame_msg(void *arg, ub_msg_id_t msg_id, const void *msg_payload,
                  int msg_payload_size);
void on_frame_end(void *arg);
} // namespace data_handle
