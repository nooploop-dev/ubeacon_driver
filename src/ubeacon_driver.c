// 分发层：把公开的 UBData* 结构与线上的字节互相转换，并驱动两个方向的解析器。
//
// 编解码只有一套：ubeacon 的 "消息 id -> 结构体" 映射与方向无关
// （配置类消息双向同构：主机 write param，设备回报同一个 PARAM）。
// 上下行的差异仅在帧 payload 头部：上行为 uid + frame_id，下行为 frame_id。
// frame_id 同时标明设备类型（见 ub_frame_id_e），本层不做过滤，交给用户回调。

#include "ubeacon_driver_for_dev.h"
#include "ubeacon_driver_for_user.h"

#include "ubeacon_driver_data_raw.h"
#include "ubeacon_frame.h"
#include "ubeacon_msg.h"

#include <string.h>

// ---------------- 帧 payload 头部 ----------------

#define UB_FRAME_PAYLOAD_TO_DEV_HEADER_SIZE 1                  // frame_id
#define UB_FRAME_PAYLOAD_TO_USER_HEADER_SIZE (UB_UID_SIZE + 1) // uid + frame_id

// ---------------- 对齐缓冲 ----------------

typedef union {
  uint8_t bytes[UB_DATA_BYTES_MAX];
  uint64_t align_;
} UBDataBuf;

typedef union {
  uint8_t bytes[UB_MSG_PAYLOAD_SIZE_MAX];
  uint64_t align_;
} UBRawBuf;

// ---------------- 扩展接口 ----------------
// 下游定义 UBEACON_DRIVER_DATA_EXTEND_ENABLED 并实现以下两个函数，
// 即可在不修改本驱动的前提下新增自定义消息。

#ifdef UBEACON_DRIVER_DATA_EXTEND_ENABLED
// 编码：把 data 转成线格式写入 raw，返回字节数；不支持的 msg_id 返回 -1
int ub_encode_extend(ub_msg_id_t msg_id, const void *data, void *raw,
                     int raw_size_max);
// 解码：把 payload 转成 UBData* 写入 data_buf，返回字节数；不支持的 msg_id 返回
// -1
int ub_decode_extend(ub_msg_id_t msg_id, const void *payload, int payload_size,
                     void *data_buf, int data_buf_size);
#endif

// ---------------- 编码：UBData* -> 消息字节 ----------------

#define UB_ENCODE(RAW_T, TO_RAW, DATA_T)                                       \
  do {                                                                         \
    _Static_assert(sizeof(RAW_T) <= UB_MSG_PAYLOAD_SIZE_MAX,                   \
                   #RAW_T " 超出单条消息 payload 上限");                       \
    TO_RAW((const DATA_T *)data, raw.bytes, &raw_size);                        \
  } while (0)

static int ub_encode(ub_msg_id_t msg_id, const void *data, void *msg_buf,
                     int msg_buf_size) {
  // data 为 NULL 表示无 payload 的消息（如 READ_* 读请求）
  if (data == NULL) {
    return ub_msg_write(msg_buf, msg_buf_size, msg_id, NULL, 0);
  }

  UBRawBuf raw;
  int raw_size = -1;

  switch (msg_id) {
  case UB_MSG_RESTART:
    UB_ENCODE(UBRawDataRestart, ub_data_restart_to_raw, UBDataRestart);
    break;
  case UB_MSG_FIND:
    UB_ENCODE(UBRawDataFind, ub_data_find_to_raw, UBDataFind);
    break;
  case UB_MSG_ANCHOR_POS:
    UB_ENCODE(UBRawDataAnchorPos, ub_data_anchor_pos_to_raw, UBDataAnchorPos);
    break;
  case UB_MSG_GLOBAL_TIME_STATUS:
    UB_ENCODE(UBRawDataGlobalTimeStatus, ub_data_global_time_status_to_raw,
              UBDataGlobalTimeStatus);
    break;
  case UB_MSG_PARAM:
    UB_ENCODE(UBRawDataParam, ub_data_param_to_raw, UBDataParam);
    break;
  case UB_MSG_INTERFACE_PARAM:
    UB_ENCODE(UBRawDataInterfaceParam, ub_data_interface_param_to_raw,
              UBDataInterfaceParam);
    break;
  case UB_MSG_UART_INTERFACE_PARAM:
    UB_ENCODE(UBRawDataUartInterfaceParam, ub_data_uart_interface_param_to_raw,
              UBDataUartInterfaceParam);
    break;
  case UB_MSG_IIC_INTERFACE_PARAM:
    UB_ENCODE(UBRawDataIicInterfaceParam, ub_data_iic_interface_param_to_raw,
              UBDataIicInterfaceParam);
    break;
  case UB_MSG_LOCATION_RESULT:
    UB_ENCODE(UBRawDataLocationResult, ub_data_location_result_to_raw,
              UBDataLocationResult);
    break;
  case UB_MSG_HEARTBEAT:
    UB_ENCODE(UBRawDataHeartbeat, ub_data_heartbeat_to_raw, UBDataHeartbeat);
    break;
  case UB_MSG_UWB_INTERFACE_PARAM:
    UB_ENCODE(UBRawDataUwbInterfaceParam, ub_data_uwb_interface_param_to_raw,
              UBDataUwbInterfaceParam);
    break;
  case UB_MSG_USER_DATA:
    UB_ENCODE(UBRawDataUserData, ub_data_user_data_to_raw, UBDataUserData);
    break;
  case UB_MSG_ANCHOR_SIGNAL:
    UB_ENCODE(UBRawDataAnchorSignal, ub_data_anchor_signal_to_raw,
              UBDataAnchorSignal);
    break;
  case UB_MSG_RUN_TIME_PARAM:
    UB_ENCODE(UBRawDataRunTimeParam, ub_data_run_time_param_to_raw,
              UBDataRunTimeParam);
    break;
  case UB_MSG_BLE_INTERFACE_PARAM:
    UB_ENCODE(UBRawDataBleInterfaceParam, ub_data_ble_interface_param_to_raw,
              UBDataBleInterfaceParam);
    break;
  case UB_MSG_ANCHOR_DDOAS:
    UB_ENCODE(UBRawDataAnchorDdoas, ub_data_anchor_ddoas_to_raw,
              UBDataAnchorDdoas);
    break;
  case UB_MSG_Z_MEASUREMENT:
    UB_ENCODE(UBRawDataZMeasurement, ub_data_z_measurement_to_raw,
              UBDataZMeasurement);
    break;
  case UB_MSG_STATE_CONTROL:
    UB_ENCODE(UBRawDataStateControl, ub_data_state_control_to_raw,
              UBDataStateControl);
    break;
  case UB_MSG_MAP_MEASUREMENT:
    UB_ENCODE(UBRawDataMapMeasurement, ub_data_map_measurement_to_raw,
              UBDataMapMeasurement);
    break;
  default:
#ifdef UBEACON_DRIVER_DATA_EXTEND_ENABLED
    raw_size =
        ub_encode_extend(msg_id, data, raw.bytes, (int)sizeof(raw.bytes));
#endif
    break;
  }

  if (raw_size < 0) {
    return -1; // 不认识的消息
  }
  return ub_msg_write(msg_buf, msg_buf_size, msg_id, raw.bytes, raw_size);
}

// ---------------- 解码：消息字节 -> UBData* ----------------

#define UB_DECODE(RAW_T, FROM_RAW, DATA_T)                                     \
  do {                                                                         \
    _Static_assert(sizeof(DATA_T) <= UB_DATA_BYTES_MAX,                        \
                   #DATA_T " 超出 UB_DATA_BYTES_MAX");                         \
    _Static_assert(sizeof(RAW_T) <= UB_MSG_PAYLOAD_SIZE_MAX,                   \
                   #RAW_T " 超出单条消息 payload 上限");                       \
    if (data_buf_size < (int)sizeof(DATA_T)) {                                 \
      return -1;                                                               \
    }                                                                          \
    RAW_T raw;                                                                 \
    /* 版本兼容：payload 多则截断，少则补零 */                                 \
    ub_msg_copy_payload(&raw, (int)sizeof(raw), payload, payload_size);        \
    /* 先清零：变长消息的 from_raw 只填充前 count 个元素，    \
       其余必须是确定值而非栈上垃圾 */                           \
    memset(data_buf, 0, sizeof(DATA_T));                                       \
    FROM_RAW(&raw, (DATA_T *)data_buf);                                        \
    data_size = (int)sizeof(DATA_T);                                           \
  } while (0)

static int ub_decode(ub_msg_id_t msg_id, const void *payload, int payload_size,
                     void *data_buf, int data_buf_size) {
  int data_size = -1;

  switch (msg_id) {
  // 读请求：无 payload
  case UB_MSG_READ_PARAM:
  case UB_MSG_READ_INTERFACE_PARAM:
  case UB_MSG_READ_UART_INTERFACE_PARAM:
  case UB_MSG_READ_IIC_INTERFACE_PARAM:
  case UB_MSG_READ_UWB_INTERFACE_PARAM:
  case UB_MSG_READ_RUN_TIME_PARAM:
  case UB_MSG_READ_BLE_INTERFACE_PARAM:
    data_size = 0;
    break;

  case UB_MSG_RESTART:
    UB_DECODE(UBRawDataRestart, ub_data_restart_from_raw, UBDataRestart);
    break;
  case UB_MSG_FIND:
    UB_DECODE(UBRawDataFind, ub_data_find_from_raw, UBDataFind);
    break;
  case UB_MSG_ANCHOR_POS:
    UB_DECODE(UBRawDataAnchorPos, ub_data_anchor_pos_from_raw, UBDataAnchorPos);
    break;
  case UB_MSG_GLOBAL_TIME_STATUS:
    UB_DECODE(UBRawDataGlobalTimeStatus, ub_data_global_time_status_from_raw,
              UBDataGlobalTimeStatus);
    break;
  case UB_MSG_PARAM:
    UB_DECODE(UBRawDataParam, ub_data_param_from_raw, UBDataParam);
    break;
  case UB_MSG_INTERFACE_PARAM:
    UB_DECODE(UBRawDataInterfaceParam, ub_data_interface_param_from_raw,
              UBDataInterfaceParam);
    break;
  case UB_MSG_UART_INTERFACE_PARAM:
    UB_DECODE(UBRawDataUartInterfaceParam,
              ub_data_uart_interface_param_from_raw, UBDataUartInterfaceParam);
    break;
  case UB_MSG_IIC_INTERFACE_PARAM:
    UB_DECODE(UBRawDataIicInterfaceParam, ub_data_iic_interface_param_from_raw,
              UBDataIicInterfaceParam);
    break;
  case UB_MSG_LOCATION_RESULT:
    UB_DECODE(UBRawDataLocationResult, ub_data_location_result_from_raw,
              UBDataLocationResult);
    break;
  case UB_MSG_HEARTBEAT:
    UB_DECODE(UBRawDataHeartbeat, ub_data_heartbeat_from_raw, UBDataHeartbeat);
    break;
  case UB_MSG_UWB_INTERFACE_PARAM:
    UB_DECODE(UBRawDataUwbInterfaceParam, ub_data_uwb_interface_param_from_raw,
              UBDataUwbInterfaceParam);
    break;
  case UB_MSG_USER_DATA:
    UB_DECODE(UBRawDataUserData, ub_data_user_data_from_raw, UBDataUserData);
    break;
  case UB_MSG_ANCHOR_SIGNAL:
    UB_DECODE(UBRawDataAnchorSignal, ub_data_anchor_signal_from_raw,
              UBDataAnchorSignal);
    break;
  case UB_MSG_RUN_TIME_PARAM:
    UB_DECODE(UBRawDataRunTimeParam, ub_data_run_time_param_from_raw,
              UBDataRunTimeParam);
    break;
  case UB_MSG_BLE_INTERFACE_PARAM:
    UB_DECODE(UBRawDataBleInterfaceParam, ub_data_ble_interface_param_from_raw,
              UBDataBleInterfaceParam);
    break;
  case UB_MSG_ANCHOR_DDOAS:
    UB_DECODE(UBRawDataAnchorDdoas, ub_data_anchor_ddoas_from_raw,
              UBDataAnchorDdoas);
    break;
  case UB_MSG_Z_MEASUREMENT:
    UB_DECODE(UBRawDataZMeasurement, ub_data_z_measurement_from_raw,
              UBDataZMeasurement);
    break;
  case UB_MSG_STATE_CONTROL:
    UB_DECODE(UBRawDataStateControl, ub_data_state_control_from_raw,
              UBDataStateControl);
    break;
  case UB_MSG_MAP_MEASUREMENT:
    UB_DECODE(UBRawDataMapMeasurement, ub_data_map_measurement_from_raw,
              UBDataMapMeasurement);
    break;
  default:
#ifdef UBEACON_DRIVER_DATA_EXTEND_ENABLED
    data_size = ub_decode_extend(msg_id, payload, payload_size, data_buf,
                                 data_buf_size);
#else
    (void)payload;
    (void)payload_size;
    (void)data_buf;
    (void)data_buf_size;
#endif
    break;
  }

  return data_size;
}

// 把一条解码后的消息交给用户回调（两个方向共用的形状）
static void dispatch_decoded_msg(const UBMsg *msg, ub_msg_id_t *out_id,
                                 UBDataBuf *data_buf, int *out_data_size) {
  *out_id = msg->id;
  *out_data_size = ub_decode(msg->id, msg->payload, msg->payload_size,
                             data_buf->bytes, (int)sizeof(data_buf->bytes));
}

// ---------------- 上行解析：设备 -> 主机 ----------------

static void parser_from_dev_on_msg(void *arg, const UBMsg *msg) {
  UBParserFromDev *parser = (UBParserFromDev *)arg;
  UBDataBuf data_buf;
  ub_msg_id_t msg_id;
  int data_size;
  dispatch_decoded_msg(msg, &msg_id, &data_buf, &data_size);
  if (data_size < 0 || !parser->on_frame_msg) {
    return; // 未识别的消息，直接忽略
  }
  parser->on_frame_msg(parser->arg, msg_id,
                       data_size > 0 ? (const void *)data_buf.bytes : NULL,
                       data_size);
}

static void parser_from_dev_on_frame(void *arg, const uint8_t *payload,
                                     int payload_size) {
  UBParserFromDev *parser = (UBParserFromDev *)arg;
  if (payload_size < UB_FRAME_PAYLOAD_TO_USER_HEADER_SIZE) {
    return;
  }
  ub_frame_id_t frame_id = payload[UB_UID_SIZE];
  // 上行有 gateway/tag/anchor 三种 frame_id，本层不做过滤：
  // 是否处理该帧(即是否来自用户所接的设备)交给 on_frame_begin 判断
  if (parser->on_frame_begin) {
    if (!parser->on_frame_begin(parser->arg, payload, frame_id)) {
      return;
    }
  }
  ub_msg_parser_handle_data(parser_from_dev_on_msg, parser,
                            payload + UB_FRAME_PAYLOAD_TO_USER_HEADER_SIZE,
                            payload_size -
                                UB_FRAME_PAYLOAD_TO_USER_HEADER_SIZE);
  if (parser->on_frame_end) {
    parser->on_frame_end(parser->arg);
  }
}

void ub_parser_from_dev_init(UBParserFromDev *parser,
                             ub_on_frame_begin_from_dev_f on_frame_begin,
                             ub_on_frame_msg_from_dev_f on_frame_msg,
                             ub_on_frame_end_from_dev_f on_frame_end,
                             void *arg) {
  ub_frame_buffer_init(&parser->frame_buffer);
  parser->on_frame_begin = on_frame_begin;
  parser->on_frame_msg = on_frame_msg;
  parser->on_frame_end = on_frame_end;
  parser->arg = arg;
}

void ub_parser_from_dev_handle_data(UBParserFromDev *parser, const void *data,
                                    int data_size) {
  ub_frame_buffer_feed(&parser->frame_buffer, data, data_size,
                       parser_from_dev_on_frame, parser);
}

// ---------------- 下行解析：主机 -> 设备 ----------------

static void parser_from_user_on_msg(void *arg, const UBMsg *msg) {
  UBParserFromUser *parser = (UBParserFromUser *)arg;
  UBDataBuf data_buf;
  ub_msg_id_t msg_id;
  int data_size;
  dispatch_decoded_msg(msg, &msg_id, &data_buf, &data_size);
  if (data_size < 0 || !parser->on_frame_msg) {
    return;
  }
  parser->on_frame_msg(parser->arg, msg_id,
                       data_size > 0 ? (const void *)data_buf.bytes : NULL,
                       data_size);
}

static void parser_from_user_on_frame(void *arg, const uint8_t *payload,
                                      int payload_size) {
  UBParserFromUser *parser = (UBParserFromUser *)arg;
  if (payload_size < UB_FRAME_PAYLOAD_TO_DEV_HEADER_SIZE) {
    return;
  }
  ub_frame_id_t frame_id = payload[0];
  // 下行只有 UB_FRAME_ID_DOWN，同样不在本层过滤，交给 on_frame_begin 判断
  if (parser->on_frame_begin) {
    if (!parser->on_frame_begin(parser->arg, frame_id)) {
      return;
    }
  }
  ub_msg_parser_handle_data(parser_from_user_on_msg, parser,
                            payload + UB_FRAME_PAYLOAD_TO_DEV_HEADER_SIZE,
                            payload_size - UB_FRAME_PAYLOAD_TO_DEV_HEADER_SIZE);
  if (parser->on_frame_end) {
    parser->on_frame_end(parser->arg);
  }
}

void ub_parser_from_user_init(UBParserFromUser *parser,
                              ub_on_frame_begin_from_user_f on_frame_begin,
                              ub_on_frame_msg_from_user_f on_frame_msg,
                              ub_on_frame_end_from_user_f on_frame_end,
                              void *arg) {
  ub_frame_buffer_init(&parser->frame_buffer);
  parser->on_frame_begin = on_frame_begin;
  parser->on_frame_msg = on_frame_msg;
  parser->on_frame_end = on_frame_end;
  parser->arg = arg;
}

void ub_parser_from_user_handle_data(UBParserFromUser *parser, const void *data,
                                     int data_size) {
  ub_frame_buffer_feed(&parser->frame_buffer, data, data_size,
                       parser_from_user_on_frame, parser);
}

// ---------------- 帧构造（两个方向共用的追加逻辑） ----------------

// 把一条消息追加到 frame 的 msg_buf 尾部。
// 构造状态就是 frame 自身的 payload_size 字段。
static bool frame_try_append_msg(ub_msg_id_t msg_id, const void *data,
                                 void *frame, int frame_size_max) {
  int payload_size = ub_frame_get_payload_size(frame);

  // 可写空间同时受 frame_size_max 与 payload 上限约束
  int space = frame_size_max - (UB_FRAME_SIZE_MIN + payload_size);
  int payload_space = UB_FRAME_PAYLOAD_SIZE_MAX - payload_size;
  if (payload_space < space) {
    space = payload_space;
  }
  if (space <= 0) {
    return false;
  }

  uint8_t *msg_buf = ub_frame_payload(frame) + payload_size;
  int msg_size = ub_encode(msg_id, data, msg_buf, space);
  if (msg_size < 0) {
    return false;
  }
  ub_frame_set_payload_size(frame, payload_size + msg_size);
  return true;
}

// ---------------- 下行构造：主机 -> 设备 ----------------

bool ub_prepare_msg_to_dev_begin(void *frame, int frame_size_max) {
  if (frame_size_max <
      UB_FRAME_SIZE_MIN + UB_FRAME_PAYLOAD_TO_DEV_HEADER_SIZE) {
    return false;
  }
  if (!ub_frame_write_begin(frame, frame_size_max)) {
    return false;
  }
  ub_frame_payload(frame)[0] = UB_FRAME_ID_DOWN;
  ub_frame_set_payload_size(frame, UB_FRAME_PAYLOAD_TO_DEV_HEADER_SIZE);
  return true;
}

bool ub_prepare_msg_to_dev_try_append(ub_msg_id_t msg_id, const void *data,
                                      void *frame, int frame_size_max) {
  return frame_try_append_msg(msg_id, data, frame, frame_size_max);
}

int ub_prepare_msg_to_dev_end(void *frame, int frame_size_max) {
  return ub_frame_write_end(frame, frame_size_max);
}

int ub_prepare_msg_to_dev(ub_msg_id_t msg_id, const void *data, void *frame,
                          int frame_size_max) {
  if (!ub_prepare_msg_to_dev_begin(frame, frame_size_max)) {
    return -1;
  }
  if (!ub_prepare_msg_to_dev_try_append(msg_id, data, frame, frame_size_max)) {
    return -1;
  }
  return ub_prepare_msg_to_dev_end(frame, frame_size_max);
}

// ---------------- 上行构造：设备 -> 主机 ----------------

bool ub_prepare_msg_to_user_begin(const uint8_t *uid, ub_frame_id_t frame_id,
                                  void *frame, int frame_size_max) {
  if (frame_size_max <
      UB_FRAME_SIZE_MIN + UB_FRAME_PAYLOAD_TO_USER_HEADER_SIZE) {
    return false;
  }
  if (!ub_frame_write_begin(frame, frame_size_max)) {
    return false;
  }
  uint8_t *payload = ub_frame_payload(frame);
  memcpy(payload, uid, UB_UID_SIZE);
  payload[UB_UID_SIZE] = frame_id;
  ub_frame_set_payload_size(frame, UB_FRAME_PAYLOAD_TO_USER_HEADER_SIZE);
  return true;
}

bool ub_prepare_msg_to_user_try_append(ub_msg_id_t msg_id, const void *data,
                                       void *frame, int frame_size_max) {
  return frame_try_append_msg(msg_id, data, frame, frame_size_max);
}

int ub_prepare_msg_to_user_end(void *frame, int frame_size_max) {
  return ub_frame_write_end(frame, frame_size_max);
}

int ub_prepare_msg_to_user(const uint8_t *uid, ub_frame_id_t frame_id,
                           ub_msg_id_t msg_id, const void *data, void *frame,
                           int frame_size_max) {
  if (!ub_prepare_msg_to_user_begin(uid, frame_id, frame, frame_size_max)) {
    return -1;
  }
  if (!ub_prepare_msg_to_user_try_append(msg_id, data, frame, frame_size_max)) {
    return -1;
  }
  return ub_prepare_msg_to_user_end(frame, frame_size_max);
}
