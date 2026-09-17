#include "main_common.hpp"
#include "serial_port.hpp"
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>

namespace {
asio::io_context g_ioc;
// 在open()而不是run()里安装信号处理：自带信号处理的库(如FTXUI)退出时会恢复
// 之前的处理函数并重新触发信号，asio先安装才能在那时接手停止io_context
std::unique_ptr<asio::signal_set> g_signals;
// g_serial_mutex 保护 g_serial 的创建、销毁与使用
std::mutex g_serial_mutex;
std::unique_ptr<SerialPort> g_serial;
} // namespace

bool main_common_open(const std::string &serial_port_name, int baud_rate,
                      UBParserFromDev *parser) {
  SerialPort::data_ready_cb_f data_ready;
  if (parser) {
    data_ready = [parser](const std::string &data) {
      ub_parser_from_dev_handle_data(parser, data.data(), (int)data.size());
    };
  }
  try {
    std::lock_guard<std::mutex> lock(g_serial_mutex);
    g_serial = std::make_unique<SerialPort>(g_ioc, serial_port_name, baud_rate,
                                            data_ready);
  } catch (const std::exception &e) {
    std::cerr << "Failed to open serial port, it may be occupied. Error: "
              << e.what() << std::endl;
    return false;
  }
  g_signals = std::make_unique<asio::signal_set>(g_ioc, SIGINT, SIGTERM);
  g_signals->async_wait([](const asio::error_code &ec, int signal_number) {
    (void)signal_number;
    if (ec == asio::error::operation_aborted) {
      return;
    }
    main_common_stop();
  });
  return true;
}

void main_common_run() {
  // 正常返回与异常退出都要释放串口
  struct Cleanup {
    ~Cleanup() {
      std::lock_guard<std::mutex> lock(g_serial_mutex);
      g_serial.reset();
      g_signals.reset();
    }
  } cleanup;
  g_ioc.run();
}

void main_common_stop() { g_ioc.stop(); }

void main_common_init(const std::string &serial_port_name, int baud_rate,
                      UBParserFromDev *parser) {
  if (!main_common_open(serial_port_name, baud_rate, parser)) {
    exit(1);
  }
  try {
    main_common_run();
  } catch (const std::exception &e) {
    std::cerr << "Serial port error: " << e.what() << std::endl;
    exit(1);
  }
}

void serial_try_send_data(const void *data, int data_size) {
  std::lock_guard<std::mutex> lock(g_serial_mutex);
  if (g_serial && data_size > 0) {
    g_serial->write(
        std::string(reinterpret_cast<const char *>(data), data_size));
  }
}

void main_common_send_msg(ub_msg_id_t msg_id, const void *msg_payload,
                          int msg_payload_size) {
  (void)msg_payload_size; // ub_prepare_msg_to_dev 由msg_id推断负载大小
  uint8_t data[UB_FRAME_SIZE_MAX];
  int data_size = ub_prepare_msg_to_dev(msg_id, msg_payload, data,
                                        sizeof(data));
  serial_try_send_data(data, data_size);
}
