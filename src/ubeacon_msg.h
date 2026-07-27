#pragma once

// 内部头文件：消息层（帧 payload 内串接的不定数量消息），用户不应包含此文件
//
// 消息格式：id(1) | payload_size(7bit) reserved(1bit) | payload(n)

#ifdef __cplusplus
extern "C" {
#endif

#include "ubeacon_driver_common.h"
#include <stddef.h>
#include <string.h>

#pragma pack(push, 1)
typedef struct {
  ub_msg_id_t id;
  uint8_t payload_size : 7;
  uint8_t reserved : 1;
  uint8_t payload[UB_MSG_PAYLOAD_SIZE_MAX];
} UBMsg;
#pragma pack(pop)

#define UB_MSG_HEADER_SIZE ((int)offsetof(UBMsg, payload))

static inline int ub_msg_size(const UBMsg *msg) {
  return UB_MSG_HEADER_SIZE + msg->payload_size;
}

typedef void (*ub_msg_cb_f)(void *arg, const UBMsg *msg);

// 遍历串接的消息。尾部不完整的消息直接丢弃，不会越界读取。
static inline void ub_msg_parser_handle_data(ub_msg_cb_f cb, void *arg,
                                             const void *data, int data_size) {
  const uint8_t *p = (const uint8_t *)data;
  int offset = 0;
  while (offset + UB_MSG_HEADER_SIZE <= data_size) {
    const UBMsg *msg = (const UBMsg *)(p + offset);
    int size = ub_msg_size(msg);
    if (offset + size > data_size) {
      break;
    }
    cb(arg, msg);
    offset += size;
  }
}

// 向 buf 写入一条消息，返回写入的字节数；参数非法或空间不足返回 -1
static inline int ub_msg_write(void *buf, int buf_size_max, ub_msg_id_t id,
                               const void *payload, int payload_size) {
  if (payload_size < 0 || payload_size > UB_MSG_PAYLOAD_SIZE_MAX) {
    return -1;
  }
  int size = UB_MSG_HEADER_SIZE + payload_size;
  if (size > buf_size_max) {
    return -1;
  }
  UBMsg *msg = (UBMsg *)buf;
  msg->id = id;
  msg->reserved = 0;
  msg->payload_size = (uint8_t)payload_size & 0x7F;
  if (payload_size > 0) {
    memcpy(msg->payload, payload, (size_t)payload_size);
  }
  return size;
}

// 版本兼容拷贝：源多则截断，源少则补零。
// 这样新固件追加尾部字段不会打挂旧解析器，旧固件少发字段也不会读到脏数据。
static inline void ub_msg_copy_payload(void *dst, int dst_size, const void *src,
                                       int src_size) {
  int n = src_size < dst_size ? src_size : dst_size;
  memset(dst, 0, (size_t)dst_size);
  if (n > 0) {
    memcpy(dst, src, (size_t)n);
  }
}

#ifdef __cplusplus
}
#endif
