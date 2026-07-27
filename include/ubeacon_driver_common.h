#pragma once

// 公共基础类型与协议常量，用户通常不需要直接包含此文件

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

// ---------------- 扩展消息钩子 ----------------
// 在不修改本驱动的前提下新增自定义消息：实现下面两种签名的函数，
// 再通过 ubeacon_driver_for_user.h / ubeacon_driver_for_dev.h 中的
// ub_set_*_extend() 注入。不注入即为不支持任何扩展消息（默认行为）。
// 钩子只在内建消息表未命中时才被调用，无法覆盖内建消息。

// 编码：把 data 转成线格式写入 raw_data，并把写入的字节数写入 *raw_data_size
typedef void (*ub_encode_extend_f)(ub_msg_id_t msg_id, const void *data,
                                   void *raw_data, int *raw_data_size);

// 解码：把 payload 转成 UBData* 写入 data_buf，返回写入的字节数（空消息返回
// 0）； 不支持的 msg_id 返回 -1。data_buf 容量不足时也应返回 -1
typedef int (*ub_decode_extend_f)(ub_msg_id_t msg_id, const void *payload,
                                  int payload_size, void *data_buf,
                                  int data_buf_size);

// 版本兼容拷贝：源多则截断，源少则补零。
// 这样新固件追加尾部字段不会打挂旧解析器，旧固件少发字段也不会读到脏数据。
static inline void ub_msg_copy_payload(void *dst, int dst_size, const void *src,
                                       int src_size) {
  int min_size = src_size < dst_size ? src_size : dst_size;
  if (min_size > 0) {
    memcpy(dst, src, min_size);
  }
  if (dst_size > src_size) {
    memset((uint8_t *)dst + src_size, 0, dst_size - src_size);
  }
}

// 编码：UBData* -> 消息字节
#define UB_MSG_DATA_ENCODE(RAW_T, TO_RAW, DATA_T)                              \
  do {                                                                         \
    _Static_assert(sizeof(RAW_T) <= UB_MSG_PAYLOAD_SIZE_MAX,                   \
                   #RAW_T " 超出单条消息 payload 上限");                       \
    TO_RAW((const DATA_T *)data, raw_data, raw_data_size);                     \
  } while (0)

// 解码：消息字节 -> UBData*
#define UB_MSG_DATA_DECODE(RAW_T, FROM_RAW, DATA_T)                            \
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

#ifdef __cplusplus
}
#endif
