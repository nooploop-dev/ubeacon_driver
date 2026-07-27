#include "main_common.hpp"
#include <CLI/CLI.hpp>
#include <chrono>
#include <csignal>
#include <cstring>
#include <limits>
#include <memory>
#include <spdlog/fmt/ranges.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

namespace {

constexpr int BAUD_RATE = 115200;

// CLI::PositiveNumber按浮点校验, 报错信息带double范围, 这里按整数校验
CLI::Validator positive_int() {
  return CLI::Range(1, std::numeric_limits<int>::max()).description("POSITIVE");
}

void send_msg(ub_msg_id_t msg_id, const void *msg_payload, const char *desc) {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  int frame_size =
      ub_prepare_msg_to_dev(msg_id, msg_payload, frame, sizeof(frame));
  if (frame_size <= 0) {
    spdlog::error("{}: prepare frame failed", desc);
    return;
  }
  serial_try_send_data(frame, frame_size);
  spdlog::info("{}:bytes={}, data={:02X}", desc, frame_size,
               fmt::join(frame, frame + frame_size, ""));
}

void fill_payload(uint8_t *payload, uint8_t *payload_size, const char *text) {
  *payload_size = (uint8_t)strlen(text);
  memcpy(payload, text, *payload_size);
}

void send_all() {
  // 等待main_common_init打开串口并启动io_context
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  {
    // 读请求无负载，msg_payload传NULL
    send_msg(UB_MSG_READ_PARAM, nullptr, "UB_MSG_READ_PARAM");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  {
    UBDataFind find{};
    find.duration = 10;
    send_msg(UB_MSG_FIND, &find, "UB_MSG_FIND");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  {
    UBDataZMeasurement z_measurement{};
    z_measurement.z = 1.75f;
    z_measurement.z_std = 0.1f;
    z_measurement.timeout = 30;
    send_msg(UB_MSG_Z_MEASUREMENT, &z_measurement, "UB_MSG_Z_MEASUREMENT");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  {
    UBDataUserData user_data{};
    fill_payload(user_data.payload, &user_data.payload_size, "user data");
    send_msg(UB_MSG_USER_DATA, &user_data, "UB_MSG_USER_DATA");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::raise(SIGINT);
}

} // namespace

int main(int argc, char **argv) {
  CLI::App app{"Assemble and send messages to a uBeacon device", APP_NAME};
  argv = app.ensure_utf8(argv);

  std::string port;
  int baud_rate = BAUD_RATE;
  app.add_option("--port", port, "serial port, e.g. /dev/ttyUSB0")->required();
  app.add_option("--baudrate", baud_rate, "serial baud rate")
      ->check(positive_int())
      ->capture_default_str();
  CLI11_PARSE(app, argc, argv);

  spdlog::info("Sending on {} at {} baud", port, baud_rate);
  std::thread sender(send_all);
  main_common_init(port, baud_rate, nullptr);
  sender.join();
  spdlog::default_logger()->flush();
  return 0;
}
