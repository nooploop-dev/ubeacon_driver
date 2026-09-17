#pragma once

#include "ubeacon_driver_for_user.h"
#include <string>

// 控制台命令：把用户输入的一行文本解析为下行消息并发送，结果通过日志反馈。
// 语法见execute()对'help'的输出
namespace console_commands {

// 记录设备回传的消息，作为后续同一消息的写命令中未指定字段的基础值。
// 可在任意线程调用
void remember(ub_msg_id_t msg_id, const void *msg_payload,
              int msg_payload_size);

enum class Result {
  CONTINUE,
  QUIT,      // 用户要求退出
  PAUSE_LOG, // 输出较长(如help)，建议暂停日志刷新以便阅读
};

// 执行一行命令
Result execute(const std::string &line);

} // namespace console_commands
