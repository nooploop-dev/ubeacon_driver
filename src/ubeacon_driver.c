// 分发层：把公开的 UBData* 结构与线上的字节互相转换，并驱动两个方向的解析器。
//
// "消息 id -> 结构体" 的映射只在单个方向内保证一一对应：同一个 msg_id 在上行与
// 下行可能对应不同的 UBData* 结构，因此编解码按方向各自一张表：
//   设备 -> 主机  ub_encode_dev_to_user / ub_decode_dev_to_user
//   主机 -> 设备  ub_encode_user_to_dev / ub_decode_user_to_dev
// 四张表相互独立，某个 msg_id 在某一方向换了结构，只需改对应方向的那张表。
//
// 上下行的差异还体现在帧 payload 头部：上行为 uid + frame_id，下行为 frame_id。
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
// 用户可选注入的钩子：与内建消息一样按方向分开，只在单个方向上用到的消息，
// 另一方向不注入或直接返回 -1 即可。
// 未注入(NULL)时行为与没有扩展消息完全一致：该 msg_id 在这个方向上不被识别。
// 主机侧的两个由 ubeacon_driver_for_user.h 注入，设备侧的由 for_dev.h 注入。

static ub_encode_extend_f s_encode_dev_to_user_extend;
static ub_encode_extend_f s_encode_user_to_dev_extend;
static ub_decode_extend_f s_decode_dev_to_user_extend;
static ub_decode_extend_f s_decode_user_to_dev_extend;

void ub_set_encode_dev_to_user_extend(ub_encode_extend_f encode) {
  s_encode_dev_to_user_extend = encode;
}
void ub_set_encode_user_to_dev_extend(ub_encode_extend_f encode) {
  s_encode_user_to_dev_extend = encode;
}
void ub_set_decode_dev_to_user_extend(ub_decode_extend_f decode) {
  s_decode_dev_to_user_extend = decode;
}
void ub_set_decode_user_to_dev_extend(ub_decode_extend_f decode) {
  s_decode_user_to_dev_extend = decode;
}

// 上行编码：设备把自己要上报的消息转成字节。对应 ubeacon_driver_data.h 中标 ^
// 的消息
static int ub_encode_dev_to_user(ub_msg_id_t msg_id, const void *data,
                                 void *msg_buf, int msg_buf_size) {

  UBRawBuf raw;
  uint8_t *raw_data = raw.bytes;
  int raw_size = -1;
  int *raw_data_size = &raw_size;

  switch (msg_id) {
  case UB_MSG_RESTART:
    UB_MSG_DATA_ENCODE(UBRawDataRestart, ub_data_restart_to_raw, UBDataRestart);
    break;
  case UB_MSG_FIND:
    UB_MSG_DATA_ENCODE(UBRawDataFind, ub_data_find_to_raw, UBDataFind);
    break;
  case UB_MSG_ANCHOR_POS:
    UB_MSG_DATA_ENCODE(UBRawDataAnchorPos, ub_data_anchor_pos_to_raw,
                       UBDataAnchorPos);
    break;
  case UB_MSG_GLOBAL_TIME_STATUS:
    UB_MSG_DATA_ENCODE(UBRawDataGlobalTimeStatus,
                       ub_data_global_time_status_to_raw,
                       UBDataGlobalTimeStatus);
    break;
  case UB_MSG_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataParam, ub_data_param_to_raw, UBDataParam);
    break;
  case UB_MSG_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataInterfaceParam, ub_data_interface_param_to_raw,
                       UBDataInterfaceParam);
    break;
  case UB_MSG_UART_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataUartInterfaceParam,
                       ub_data_uart_interface_param_to_raw,
                       UBDataUartInterfaceParam);
    break;
  case UB_MSG_IIC_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataIicInterfaceParam,
                       ub_data_iic_interface_param_to_raw,
                       UBDataIicInterfaceParam);
    break;
  case UB_MSG_LOCATION_RESULT:
    UB_MSG_DATA_ENCODE(UBRawDataLocationResult, ub_data_location_result_to_raw,
                       UBDataLocationResult);
    break;
  case UB_MSG_HEARTBEAT:
    UB_MSG_DATA_ENCODE(UBRawDataHeartbeat, ub_data_heartbeat_to_raw,
                       UBDataHeartbeat);
    break;
  case UB_MSG_UWB_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataUwbInterfaceParam,
                       ub_data_uwb_interface_param_to_raw,
                       UBDataUwbInterfaceParam);
    break;
  case UB_MSG_USER_DATA:
    UB_MSG_DATA_ENCODE(UBRawDataUserData, ub_data_user_data_to_raw,
                       UBDataUserData);
    break;
  case UB_MSG_ANCHOR_SIGNAL:
    UB_MSG_DATA_ENCODE(UBRawDataAnchorSignal, ub_data_anchor_signal_to_raw,
                       UBDataAnchorSignal);
    break;
  case UB_MSG_RUN_TIME_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataRunTimeParam, ub_data_run_time_param_to_raw,
                       UBDataRunTimeParam);
    break;
  case UB_MSG_BLE_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataBleInterfaceParam,
                       ub_data_ble_interface_param_to_raw,
                       UBDataBleInterfaceParam);
    break;
  case UB_MSG_ANCHOR_DDOAS:
    UB_MSG_DATA_ENCODE(UBRawDataAnchorDdoas, ub_data_anchor_ddoas_to_raw,
                       UBDataAnchorDdoas);
    break;
  case UB_MSG_Z_MEASUREMENT:
    UB_MSG_DATA_ENCODE(UBRawDataZMeasurement, ub_data_z_measurement_to_raw,
                       UBDataZMeasurement);
    break;
  case UB_MSG_STATE_CONTROL:
    UB_MSG_DATA_ENCODE(UBRawDataStateControl, ub_data_state_control_to_raw,
                       UBDataStateControl);
    break;
  case UB_MSG_MAP_MEASUREMENT:
    UB_MSG_DATA_ENCODE(UBRawDataMapMeasurement, ub_data_map_measurement_to_raw,
                       UBDataMapMeasurement);
    break;
  default:
    if (s_encode_dev_to_user_extend) {
      s_encode_dev_to_user_extend(msg_id, data, raw_data, raw_data_size);
    }
    break;
  }

  if (*raw_data_size < 0) {
    return -1; // 该方向上不认识的消息
  }
  return ub_msg_write(msg_buf, msg_buf_size, msg_id, raw_data, *raw_data_size);
}

// 下行编码：主机把要下发的命令转成字节。对应 ubeacon_driver_data.h 中标 v
// 的消息
static int ub_encode_user_to_dev(ub_msg_id_t msg_id, const void *data,
                                 void *msg_buf, int msg_buf_size) {

  UBRawBuf raw;
  uint8_t *raw_data = raw.bytes;
  int raw_size = -1;
  int *raw_data_size = &raw_size;

  switch (msg_id) {
  case UB_MSG_READ_PARAM:
  case UB_MSG_READ_INTERFACE_PARAM:
  case UB_MSG_READ_UART_INTERFACE_PARAM:
  case UB_MSG_READ_IIC_INTERFACE_PARAM:
  case UB_MSG_READ_UWB_INTERFACE_PARAM:
  case UB_MSG_READ_RUN_TIME_PARAM:
  case UB_MSG_READ_BLE_INTERFACE_PARAM:
    // 读请求等空消息只写消息头
    *raw_data_size = 0;
    break;
  case UB_MSG_RESTART:
    UB_MSG_DATA_ENCODE(UBRawDataRestart, ub_data_restart_to_raw, UBDataRestart);
    break;
  case UB_MSG_FIND:
    UB_MSG_DATA_ENCODE(UBRawDataFind, ub_data_find_to_raw, UBDataFind);
    break;
  case UB_MSG_WRITE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataParam, ub_data_param_to_raw, UBDataParam);
    break;
  case UB_MSG_WRITE_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataInterfaceParam, ub_data_interface_param_to_raw,
                       UBDataInterfaceParam);
    break;
  case UB_MSG_WRITE_UART_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataUartInterfaceParam,
                       ub_data_uart_interface_param_to_raw,
                       UBDataUartInterfaceParam);
    break;
  case UB_MSG_WRITE_IIC_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataIicInterfaceParam,
                       ub_data_iic_interface_param_to_raw,
                       UBDataIicInterfaceParam);
    break;
  case UB_MSG_WRITE_UWB_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataUwbInterfaceParam,
                       ub_data_uwb_interface_param_to_raw,
                       UBDataUwbInterfaceParam);
    break;
  case UB_MSG_USER_DATA:
    UB_MSG_DATA_ENCODE(UBRawDataUserData, ub_data_user_data_to_raw,
                       UBDataUserData);
    break;
  case UB_MSG_WRITE_RUN_TIME_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataRunTimeParam, ub_data_run_time_param_to_raw,
                       UBDataRunTimeParam);
    break;
  case UB_MSG_WRITE_BLE_INTERFACE_PARAM:
    UB_MSG_DATA_ENCODE(UBRawDataBleInterfaceParam,
                       ub_data_ble_interface_param_to_raw,
                       UBDataBleInterfaceParam);
    break;
  case UB_MSG_Z_MEASUREMENT:
    UB_MSG_DATA_ENCODE(UBRawDataZMeasurement, ub_data_z_measurement_to_raw,
                       UBDataZMeasurement);
    break;
  case UB_MSG_STATE_CONTROL:
    UB_MSG_DATA_ENCODE(UBRawDataStateControl, ub_data_state_control_to_raw,
                       UBDataStateControl);
    break;
  case UB_MSG_MAP_MEASUREMENT:
    UB_MSG_DATA_ENCODE(UBRawDataMapMeasurement, ub_data_map_measurement_to_raw,
                       UBDataMapMeasurement);
    break;
  default:
    if (s_encode_user_to_dev_extend) {
      s_encode_user_to_dev_extend(msg_id, data, raw_data, raw_data_size);
    }
    break;
  }
  if (*raw_data_size < 0) {
    return -1; // 该方向上不认识的消息
  }
  return ub_msg_write(msg_buf, msg_buf_size, msg_id, raw_data, *raw_data_size);
}

// 上行解码：主机解析设备上报的消息。对应 ubeacon_driver_data.h 中标 ^ 的消息
static int ub_decode_dev_to_user(ub_msg_id_t msg_id, const void *payload,
                                 int payload_size, void *data_buf,
                                 int data_buf_size) {
  int data_size = -1;

  switch (msg_id) {
  case UB_MSG_RESTART:
    UB_MSG_DATA_DECODE(UBRawDataRestart, ub_data_restart_from_raw,
                       UBDataRestart);
    break;
  case UB_MSG_FIND:
    UB_MSG_DATA_DECODE(UBRawDataFind, ub_data_find_from_raw, UBDataFind);
    break;
  case UB_MSG_ANCHOR_POS:
    UB_MSG_DATA_DECODE(UBRawDataAnchorPos, ub_data_anchor_pos_from_raw,
                       UBDataAnchorPos);
    break;
  case UB_MSG_GLOBAL_TIME_STATUS:
    UB_MSG_DATA_DECODE(UBRawDataGlobalTimeStatus,
                       ub_data_global_time_status_from_raw,
                       UBDataGlobalTimeStatus);
    break;
  case UB_MSG_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataParam, ub_data_param_from_raw, UBDataParam);
    break;
  case UB_MSG_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataInterfaceParam,
                       ub_data_interface_param_from_raw, UBDataInterfaceParam);
    break;
  case UB_MSG_UART_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataUartInterfaceParam,
                       ub_data_uart_interface_param_from_raw,
                       UBDataUartInterfaceParam);
    break;
  case UB_MSG_IIC_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataIicInterfaceParam,
                       ub_data_iic_interface_param_from_raw,
                       UBDataIicInterfaceParam);
    break;
  case UB_MSG_LOCATION_RESULT:
    UB_MSG_DATA_DECODE(UBRawDataLocationResult,
                       ub_data_location_result_from_raw, UBDataLocationResult);
    break;
  case UB_MSG_HEARTBEAT:
    UB_MSG_DATA_DECODE(UBRawDataHeartbeat, ub_data_heartbeat_from_raw,
                       UBDataHeartbeat);
    break;
  case UB_MSG_UWB_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataUwbInterfaceParam,
                       ub_data_uwb_interface_param_from_raw,
                       UBDataUwbInterfaceParam);
    break;
  case UB_MSG_USER_DATA:
    UB_MSG_DATA_DECODE(UBRawDataUserData, ub_data_user_data_from_raw,
                       UBDataUserData);
    break;
  case UB_MSG_ANCHOR_SIGNAL:
    UB_MSG_DATA_DECODE(UBRawDataAnchorSignal, ub_data_anchor_signal_from_raw,
                       UBDataAnchorSignal);
    break;
  case UB_MSG_RUN_TIME_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataRunTimeParam, ub_data_run_time_param_from_raw,
                       UBDataRunTimeParam);
    break;
  case UB_MSG_BLE_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataBleInterfaceParam,
                       ub_data_ble_interface_param_from_raw,
                       UBDataBleInterfaceParam);
    break;
  case UB_MSG_ANCHOR_DDOAS:
    UB_MSG_DATA_DECODE(UBRawDataAnchorDdoas, ub_data_anchor_ddoas_from_raw,
                       UBDataAnchorDdoas);
    break;
  case UB_MSG_Z_MEASUREMENT:
    UB_MSG_DATA_DECODE(UBRawDataZMeasurement, ub_data_z_measurement_from_raw,
                       UBDataZMeasurement);
    break;
  case UB_MSG_STATE_CONTROL:
    UB_MSG_DATA_DECODE(UBRawDataStateControl, ub_data_state_control_from_raw,
                       UBDataStateControl);
    break;
  case UB_MSG_MAP_MEASUREMENT:
    UB_MSG_DATA_DECODE(UBRawDataMapMeasurement,
                       ub_data_map_measurement_from_raw, UBDataMapMeasurement);
    break;
  default:
    if (s_decode_dev_to_user_extend) {
      data_size = s_decode_dev_to_user_extend(msg_id, payload, payload_size,
                                              data_buf, data_buf_size);
    }
    break;
  }

  return data_size;
}

// 下行解码：设备解析主机下发的命令。对应 ubeacon_driver_data.h 中标 v 的消息
static int ub_decode_user_to_dev(ub_msg_id_t msg_id, const void *payload,
                                 int payload_size, void *data_buf,
                                 int data_buf_size) {

  int data_size = -1;

  switch (msg_id) {
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
    UB_MSG_DATA_DECODE(UBRawDataRestart, ub_data_restart_from_raw,
                       UBDataRestart);
    break;
  case UB_MSG_FIND:
    UB_MSG_DATA_DECODE(UBRawDataFind, ub_data_find_from_raw, UBDataFind);
    break;
  case UB_MSG_WRITE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataParam, ub_data_param_from_raw, UBDataParam);
    break;
  case UB_MSG_WRITE_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataInterfaceParam,
                       ub_data_interface_param_from_raw, UBDataInterfaceParam);
    break;
  case UB_MSG_WRITE_UART_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataUartInterfaceParam,
                       ub_data_uart_interface_param_from_raw,
                       UBDataUartInterfaceParam);
    break;
  case UB_MSG_WRITE_IIC_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataIicInterfaceParam,
                       ub_data_iic_interface_param_from_raw,
                       UBDataIicInterfaceParam);
    break;
  case UB_MSG_WRITE_UWB_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataUwbInterfaceParam,
                       ub_data_uwb_interface_param_from_raw,
                       UBDataUwbInterfaceParam);
    break;
  case UB_MSG_USER_DATA:
    UB_MSG_DATA_DECODE(UBRawDataUserData, ub_data_user_data_from_raw,
                       UBDataUserData);
    break;
  case UB_MSG_WRITE_RUN_TIME_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataRunTimeParam, ub_data_run_time_param_from_raw,
                       UBDataRunTimeParam);
    break;
  case UB_MSG_WRITE_BLE_INTERFACE_PARAM:
    UB_MSG_DATA_DECODE(UBRawDataBleInterfaceParam,
                       ub_data_ble_interface_param_from_raw,
                       UBDataBleInterfaceParam);
    break;
  case UB_MSG_Z_MEASUREMENT:
    UB_MSG_DATA_DECODE(UBRawDataZMeasurement, ub_data_z_measurement_from_raw,
                       UBDataZMeasurement);
    break;
  case UB_MSG_STATE_CONTROL:
    UB_MSG_DATA_DECODE(UBRawDataStateControl, ub_data_state_control_from_raw,
                       UBDataStateControl);
    break;
  case UB_MSG_MAP_MEASUREMENT:
    UB_MSG_DATA_DECODE(UBRawDataMapMeasurement,
                       ub_data_map_measurement_from_raw, UBDataMapMeasurement);
    break;
  default:
    if (s_decode_user_to_dev_extend) {
      data_size = s_decode_user_to_dev_extend(msg_id, payload, payload_size,
                                              data_buf, data_buf_size);
    }
    break;
  }

  return data_size;
}

// ---------------- 上行解析：设备 -> 主机 ----------------

static void parser_from_dev_on_msg(void *arg, const UBMsg *msg) {
  UBParserFromDev *parser = (UBParserFromDev *)arg;
  UBDataBuf data_buf;
  int data_size =
      ub_decode_dev_to_user(msg->id, msg->payload, msg->payload_size,
                            data_buf.bytes, (int)sizeof(data_buf.bytes));
  if (data_size < 0 || !parser->on_frame_msg) {
    return; // 未识别的消息，直接忽略
  }
  parser->on_frame_msg(parser->arg, msg->id,
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
  int data_size =
      ub_decode_user_to_dev(msg->id, msg->payload, msg->payload_size,
                            data_buf.bytes, (int)sizeof(data_buf.bytes));
  if (data_size < 0 || !parser->on_frame_msg) {
    return;
  }
  parser->on_frame_msg(parser->arg, msg->id,
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

// 编码器由调用方按方向选定
typedef int (*ub_encode_f)(ub_msg_id_t msg_id, const void *data, void *msg_buf,
                           int msg_buf_size);

// 把一条消息追加到 frame 的 msg_buf 尾部。
// 构造状态就是 frame 自身的 payload_size 字段。
static bool frame_try_append_msg(ub_encode_f encode, ub_msg_id_t msg_id,
                                 const void *data, void *frame,
                                 int frame_size_max) {
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
  int msg_size = encode(msg_id, data, msg_buf, space);
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
  return frame_try_append_msg(ub_encode_user_to_dev, msg_id, data, frame,
                              frame_size_max);
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
  return frame_try_append_msg(ub_encode_dev_to_user, msg_id, data, frame,
                              frame_size_max);
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
