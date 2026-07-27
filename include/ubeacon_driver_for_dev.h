#pragma once

// 设备侧驱动接口：接收主机下行命令 / 构造上行数据
// 与 ubeacon_driver_for_user.h 完全对称，供设备固件或设备仿真器使用。
// 两侧复用同一套编解码与转换器，因此测试可做完整的双向 round-trip。

#ifdef __cplusplus
extern "C" {
#endif

#include "ubeacon_driver_common.h"
#include "ubeacon_driver_data.h"

// ---------------- 下行：主机 -> 设备（设备接收） ----------------

// 一帧开始。frame_id 为帧 payload 首部的类型标识（取值见 ub_frame_id_e），
// 主机下行统一为 UB_FRAME_ID_DOWN。
// 返回 true 才会继续解析该帧内的消息（并在结束时回调 on_frame_end），
// 返回 false 则整帧丢弃。
// 驱动自身不按 frame_id 过滤：未注册该回调时，任何帧都会被按下行帧解析。
// 若收发线路上可能出现上行帧（上下行的 payload 头部长度不同），务必注册此回调
// 并只对 UB_FRAME_ID_DOWN 返回 true
typedef bool (*ub_on_frame_begin_from_user_f)(void *arg,
                                              ub_frame_id_t frame_id);

// 帧内的一条消息。data 指向对应的 UBData* 结构；
// 对于 READ_* 请求等无 payload 的消息，data 为 NULL 且 data_size 为 0
typedef void (*ub_on_frame_msg_from_user_f)(void *arg, ub_msg_id_t msg_id,
                                            const void *data, int data_size);

typedef void (*ub_on_frame_end_from_user_f)(void *arg);

typedef struct {
  UBFrameBuffer frame_buffer;
  ub_on_frame_begin_from_user_f on_frame_begin;
  ub_on_frame_msg_from_user_f on_frame_msg;
  ub_on_frame_end_from_user_f on_frame_end;
  void *arg;
} UBParserFromUser;

void ub_parser_from_user_init(UBParserFromUser *parser,
                              ub_on_frame_begin_from_user_f on_frame_begin,
                              ub_on_frame_msg_from_user_f on_frame_msg,
                              ub_on_frame_end_from_user_f on_frame_end,
                              void *arg);

void ub_parser_from_user_handle_data(UBParserFromUser *parser, const void *data,
                                     int data_size);

// ---------------- 上行：设备 -> 主机（设备发送） ----------------
// 与下行的区别仅在于帧 payload 头部额外携带设备 uid
//
// frame_id 标明本设备的类型，由固件按自身角色固定填写：
// UB_FRAME_ID_GATEWAY_UP / UB_FRAME_ID_TAG_UP / UB_FRAME_ID_ANCHOR_UP

int ub_prepare_msg_to_user(const uint8_t *uid, ub_frame_id_t frame_id,
                           ub_msg_id_t msg_id, const void *data, void *frame,
                           int frame_size_max);

bool ub_prepare_msg_to_user_begin(const uint8_t *uid, ub_frame_id_t frame_id,
                                  void *frame, int frame_size_max);
bool ub_prepare_msg_to_user_try_append(ub_msg_id_t msg_id, const void *data,
                                       void *frame, int frame_size_max);
int ub_prepare_msg_to_user_end(void *frame, int frame_size_max);

#ifdef __cplusplus
}
#endif
