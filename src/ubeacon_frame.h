#pragma once

// 内部头文件：帧层（字节流分包与重同步），用户不应包含此文件
//
// 帧格式：sof(1) | payload_size(2, 小端) | payload(n) | checksum(1)
// 上下行帧头结构一致，方向差异体现在 payload 内部（见 ubeacon_driver.c），
// 因此帧层与方向无关。

#ifdef __cplusplus
extern "C" {
#endif

#include "ubeacon_driver_common.h"

typedef uint8_t ub_frame_sof_t;
typedef uint16_t ub_frame_payload_size_t;

// payload 在帧内的起始偏移
#define UB_FRAME_PAYLOAD_OFFSET                                                \
  ((int)(sizeof(ub_frame_sof_t) + sizeof(ub_frame_payload_size_t)))

// ---------------- 帧解析 ----------------

// 提取出一个完整且校验通过的帧时回调
typedef void (*ub_frame_cb_f)(void *arg, const uint8_t *payload,
                              int payload_size);

// UBFrameBuffer 定义在 ubeacon_driver_common.h（供 parser 内联分配）
void ub_frame_buffer_init(UBFrameBuffer *fb);

// 喂入任意分片/黏包的字节流；每提取出一个合法帧就回调一次。
// 任何不匹配（sof 错误、长度越界、校验失败）都只丢弃一个字节后重新同步。
void ub_frame_buffer_feed(UBFrameBuffer *fb, const void *data, int data_size,
                          ub_frame_cb_f cb, void *arg);

// ---------------- 帧构造 ----------------
// 构造过程的状态就存放在 frame 缓冲自身的 payload_size 字段中，无需额外对象

// 读写 frame 的 payload_size 字段（可能非对齐，故用 memcpy）
int ub_frame_get_payload_size(const void *frame);
void ub_frame_set_payload_size(void *frame, int payload_size);

static inline uint8_t *ub_frame_payload(void *frame) {
  return (uint8_t *)frame + UB_FRAME_PAYLOAD_OFFSET;
}

// 写入 sof 并将 payload_size 清零。frame_size_max 不足时返回 false
bool ub_frame_write_begin(void *frame, int frame_size_max);

// 写入校验和，返回总帧长；frame_size_max 不足时返回 -1
int ub_frame_write_end(void *frame, int frame_size_max);

#ifdef __cplusplus
}
#endif
