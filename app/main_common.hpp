#pragma once

#include "ubeacon_driver_for_user.h"
#include <string>

// 打开串口并驱动parser处理收到的数据，阻塞直到退出
void main_common_init(const std::string &serial_port_name, int baud_rate,
                      UBParserFromDev *parser);

// 尝试通过串口发送数据
void serial_try_send_data(const void *data, int data_size);

// 向设备发送一条消息(内部完成组帧并通过串口发出)
void main_common_send_msg(ub_msg_id_t msg_id, const void *msg_payload,
                          int msg_payload_size);
