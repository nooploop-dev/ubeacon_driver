#pragma once

// 公共基础类型与协议常量，用户通常不需要直接包含此文件

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// 设备通信短地址
typedef uint16_t ub_addr_t;

// 设备本地时间，单位 us，复位后从 0 开始
typedef int64_t ub_local_time_us_t;

// 设备唯一 id 长度
#define UB_UID_SIZE 6

// 串口为字节流数据，需要额外的帧格式用于分包
// 帧格式：sof(1) | payload_size(2, 小端) | payload(n) | checksum(1)

#define UB_FRAME_SOF 0xAA
#define UB_FRAME_SIZE_MIN 4
#define UB_FRAME_SIZE_MAX 1024
#define UB_FRAME_PAYLOAD_SIZE_MAX (UB_FRAME_SIZE_MAX - UB_FRAME_SIZE_MIN)

// 帧 id，即帧 payload 首部标识数据方向/类型的字节
typedef uint8_t ub_frame_id_t;
// 下行：主机 -> 设备，payload = frame_id + msg_buf
// 上行：设备 -> 主机，payload = uid + frame_id + msg_buf
typedef enum {
  UB_FRAME_ID_INVALID,
  // gateway上行
  UB_FRAME_ID_GATEWAY_UP,
  // 对所有设备的通用下行
  UB_FRAME_ID_DOWN,
  UB_FRAME_ID_RESERVED0,
  UB_FRAME_ID_RESERVED1,
  // tag上行
  UB_FRAME_ID_TAG_UP,
  UB_FRAME_ID_RESERVED2,
  UB_FRAME_ID_RESERVED3,
  // anchor上行
  UB_FRAME_ID_ANCHOR_UP,
} ub_frame_id_e;

// 字节流分包缓冲
typedef struct {
  uint8_t buffer[UB_FRAME_SIZE_MAX];
  int index_begin;
  int index_end;
} UBFrameBuffer;

// 消息 id
typedef uint8_t ub_msg_id_t;

// 一个帧的 msg_buf 中可串接不定数量的消息
// 消息格式：id(1) | payload_size(7bit) | payload(n)

// 单条消息 payload 上限
#define UB_MSG_PAYLOAD_SIZE_MAX 127

// 解码后 UBData* 结构的最大字节数，由 ubeacon_driver.c 中的静态断言守护
#define UB_DATA_BYTES_MAX 512

#ifdef __cplusplus
}
#endif
