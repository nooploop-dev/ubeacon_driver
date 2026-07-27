#pragma once

// 主机侧（用户侧）驱动接口：接收设备上行数据 / 构造下行命令
// 这是主机应用需要包含的唯一头文件。
// 用户只接触 UBData* 结构与消息id，无需接触字节、位域、校验和与字节序等协议细节

#ifdef __cplusplus
extern "C" {
#endif

#include "ubeacon_driver_common.h"
#include "ubeacon_driver_data.h"

// ---------------- 上行：设备 -> 主机（主机接收） ----------------

// 一帧开始。uid 为发送该帧的设备唯一 id，长度 UB_UID_SIZE；
// frame_id 为帧 payload 首部的类型标识（取值见 ub_frame_id_e），标明该帧来自
// 哪类设备：UB_FRAME_ID_GATEWAY_UP / UB_FRAME_ID_TAG_UP /
// UB_FRAME_ID_ANCHOR_UP。
// 返回 true 才会继续解析该帧内的消息（并在结束时回调 on_frame_end），
// 返回 false 则整帧丢弃 —— 由用户据此判断该帧是否来自自己所接的设备。
// 驱动自身不按 frame_id 过滤：未注册该回调时，任何帧都会被按上行帧解析
typedef bool (*ub_on_frame_begin_from_dev_f)(void *arg, const uint8_t *uid,
                                             ub_frame_id_t frame_id);

// 帧内的一条消息。data 指向对应的 UBData* 结构（由 msg_id 决定具体类型）；
// 对于 READ_* 请求等无 payload 的消息，data 为 NULL 且 data_size 为 0
typedef void (*ub_on_frame_msg_from_dev_f)(void *arg, ub_msg_id_t msg_id,
                                           const void *data, int data_size);

// 一帧结束
typedef void (*ub_on_frame_end_from_dev_f)(void *arg);

typedef struct {
  UBFrameBuffer frame_buffer;
  ub_on_frame_begin_from_dev_f on_frame_begin;
  ub_on_frame_msg_from_dev_f on_frame_msg;
  ub_on_frame_end_from_dev_f on_frame_end;
  void *arg;
} UBParserFromDev;

// 三个回调均可为 NULL。arg 会原样传回给回调
void ub_parser_from_dev_init(UBParserFromDev *parser,
                             ub_on_frame_begin_from_dev_f on_frame_begin,
                             ub_on_frame_msg_from_dev_f on_frame_msg,
                             ub_on_frame_end_from_dev_f on_frame_end,
                             void *arg);

// 喂入串口等字节流，内部完成分包、校验、解析与解码
void ub_parser_from_dev_handle_data(UBParserFromDev *parser, const void *data,
                                    int data_size);

// ---------------- 下行：主机 -> 设备（主机发送） ----------------

// 构造只含一条消息的帧，返回帧长度；失败返回 -1。
// data 指向 msg_id 对应的 UBData* 结构；READ_* 等无 payload 的消息传 NULL
int ub_prepare_msg_to_dev(ub_msg_id_t msg_id, const void *data, void *frame,
                          int frame_size_max);

// 一帧多消息：begin -> try_append（可重复）-> end。
// 构造过程的状态保存在 frame 缓冲自身，无需额外对象
bool ub_prepare_msg_to_dev_begin(void *frame, int frame_size_max);
bool ub_prepare_msg_to_dev_try_append(ub_msg_id_t msg_id, const void *data,
                                      void *frame, int frame_size_max);
// 返回最终帧长度；失败返回 -1
int ub_prepare_msg_to_dev_end(void *frame, int frame_size_max);

#ifdef __cplusplus
}
#endif
