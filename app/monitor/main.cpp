#include "console_commands.hpp"
#include "foxglove_viz.hpp"
#include "main_common.hpp"
#include "monitor_data_handle.hpp"
#include "tui_console.hpp"
#include <CLI/CLI.hpp>
#include <chrono>
#include <filesystem>
#include <limits>
#include <memory>
#include <spdlog/details/os.h>
#include <spdlog/fmt/chrono.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

namespace {
constexpr char LOG_DIR[] = "logs";
constexpr char LOG_FILE[] = "logs/logger.log"; // 与录制文件同在LOG_DIR下
constexpr std::size_t MAX_FILE_SIZE = 10 * 1024 * 1024;
constexpr std::size_t MAX_FILE_COUNT = 3;
constexpr int BAUD_RATE = 115200;
// Foxglove可视化与MCAP录制(仅在开启UB_BUILD_FOXGLOVE时生效)
constexpr char FOXGLOVE_HOST[] = "0.0.0.0";
constexpr uint16_t FOXGLOVE_PORT = 8765;
constexpr char FOXGLOVE_FRAME_ID[] = "map";

// CLI::PositiveNumber按浮点校验, 报错信息带double范围, 这里按整数校验
CLI::Validator positive_int() {
  return CLI::Range(1, std::numeric_limits<int>::max()).description("POSITIVE");
}

// 录制文件路径，形如logs/ubeacon_20260713_182432_123.mcap。
std::string record_file_path() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  // fmt::localtime已废弃, std::localtime非线程安全, 用spdlog的跨平台实现
  const std::tm tm =
      spdlog::details::os::localtime(system_clock::to_time_t(now));
  return fmt::format("{}/ubeacon_{:%Y%m%d_%H%M%S}_{:03}.mcap", LOG_DIR, tm,
                     ms.count());
}

// 控制台模式下额外记录设备回传，供写命令作为基础值
void on_frame_msg_with_console(void *arg, ub_msg_id_t msg_id,
                               const void *msg_payload, int msg_payload_size) {
  console_commands::remember(msg_id, msg_payload, msg_payload_size);
  data_handle::on_frame_msg(arg, msg_id, msg_payload, msg_payload_size);
}
} // namespace

int main(int argc, char **argv) {
  CLI::App app{"Monitor a uBeacon device and send it commands", APP_NAME};
  argv = app.ensure_utf8(argv);

  std::string port;
  int baud_rate = BAUD_RATE;
  uint16_t foxglove_port = FOXGLOVE_PORT;
  bool record = false;
  bool no_console = false;
  app.add_option("--port", port, "serial port, e.g. /dev/ttyUSB0")->required();
  app.add_option("--baudrate", baud_rate, "serial baud rate")
      ->check(positive_int())
      ->capture_default_str();
  app.add_option("--foxglove_port", foxglove_port, "Foxglove WebSocket port")
      ->check(CLI::Range(1, 65535))
      ->capture_default_str();
  app.add_flag("--record", record,
               "record data to logs/ubeacon_<date>.mcap for later replay");
  app.add_flag("--no-console", no_console,
               "print logs to stdout instead of the interactive console "
               "(implied when stdout is not a terminal)");
  CLI11_PARSE(app, argc, argv);
  const bool console = !no_console && spdlog::details::os::in_terminal(stdout);

  {
    std::vector<spdlog::sink_ptr> sinks;
    if (console) {
      sinks.push_back(tui_console::make_sink());
    } else {
      sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }
    try {
      sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          LOG_FILE, MAX_FILE_SIZE, MAX_FILE_COUNT));
    } catch (const spdlog::spdlog_ex &e) {
      spdlog::warn("Failed to create log file '{}': {}", LOG_FILE, e.what());
    }
    auto logger =
        std::make_shared<spdlog::logger>("", sinks.begin(), sinks.end());
    spdlog::set_default_logger(logger);
    spdlog::flush_every(std::chrono::seconds(1));
  }

  UBParserFromDev parser;
  ub_parser_from_dev_init(&parser, data_handle::on_frame_begin,
                          console ? on_frame_msg_with_console
                                  : data_handle::on_frame_msg,
                          data_handle::on_frame_end, nullptr);
  // 先开串口，失败时错误直接打印到终端，不进入控制台界面
  if (!main_common_open(port, baud_rate, &parser)) {
    return 1;
  }
  spdlog::info("Listening on {} at {} baud", port, baud_rate);
  if (foxglove_viz::init(FOXGLOVE_HOST, foxglove_port, FOXGLOVE_FRAME_ID)) {
    spdlog::info("Foxglove: connect to ws://{}:{}", FOXGLOVE_HOST,
                 foxglove_port);
  }
  if (record) {
    // MCAP写入器不会自动建目录, 而日志sink建目录失败时只告警不退出
    std::error_code ec;
    std::filesystem::create_directories(LOG_DIR, ec);
    const std::string file = record_file_path();
    if (foxglove_viz::record_start(file)) {
      spdlog::info("Record: writing to {}", file);
    }
  }

  std::string serial_error;
  auto run_serial = [&serial_error] {
    try {
      main_common_run();
    } catch (const std::exception &e) {
      serial_error = e.what();
    }
  };
  if (console) {
    // 串口在后台线程收发，主线程运行界面；任一方结束都让另一方退出
    std::thread serial_thread([&run_serial] {
      run_serial();
      tui_console::request_exit();
    });
    tui_console::run([](const std::string &line) {
      switch (console_commands::execute(line)) {
      case console_commands::Result::QUIT:
        tui_console::request_exit();
        break;
      case console_commands::Result::PAUSE_LOG:
        tui_console::pause();
        break;
      case console_commands::Result::CONTINUE:
        break;
      }
    });
    main_common_stop();
    serial_thread.join();
  } else {
    run_serial();
  }
  if (!serial_error.empty()) {
    spdlog::error("Serial port error: {}", serial_error);
  }

  foxglove_viz::shutdown();
  spdlog::info("Exiting");
  spdlog::default_logger()->flush();
  return serial_error.empty() ? 0 : 1;
}
