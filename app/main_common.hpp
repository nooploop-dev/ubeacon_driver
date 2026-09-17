#pragma once

#include "ubeacon_driver_for_user.h"
#include <string>

// 打开串口并注册SIGINT/SIGTERM退出处理，失败时输出错误并返回false。
// 收到的数据交给parser解析，parser可为NULL
bool main_common_open(const std::string &serial_port_name, int baud_rate,
                      UBParserFromDev *parser);

// 驱动串口收发，阻塞直到main_common_stop()或收到SIGINT/SIGTERM；
// 串口出错时抛出std::runtime_error。可在任意线程调用
void main_common_run();

// 结束main_common_run()的阻塞，可在任意线程调用
void main_common_stop();

// 打开串口并驱动parser处理收到的数据，阻塞直到退出；出错时exit(1)
void main_common_init(const std::string &serial_port_name, int baud_rate,
                      UBParserFromDev *parser);

// 尝试通过串口发送数据
void serial_try_send_data(const void *data, int data_size);

// 向设备发送一条消息(内部完成组帧并通过串口发出)
void main_common_send_msg(ub_msg_id_t msg_id, const void *msg_payload,
                          int msg_payload_size);
