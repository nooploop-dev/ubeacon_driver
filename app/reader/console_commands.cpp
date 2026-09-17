#include "console_commands.hpp"
#include "main_common.hpp"

#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <spdlog/fmt/ranges.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <type_traits>
#include <vector>

namespace {

// 字段值解析：bool接受0/1/true/false，整数接受十进制或0x十六进制并做范围检查
bool parse_value(const std::string &s, bool &out) {
  if (s == "1" || s == "true") {
    out = true;
    return true;
  }
  if (s == "0" || s == "false") {
    out = false;
    return true;
  }
  return false;
}

bool parse_value(const std::string &s, float &out) {
  char *end = nullptr;
  errno = 0;
  const float value = std::strtof(s.c_str(), &end);
  if (s.empty() || *end != '\0' || errno == ERANGE) {
    return false;
  }
  out = value;
  return true;
}

template <typename T> bool parse_value(const std::string &s, T &out) {
  static_assert(std::is_integral<T>::value, "integer field expected");
  char *end = nullptr;
  errno = 0;
  const long long value = std::strtoll(s.c_str(), &end, 0);
  if (s.empty() || *end != '\0' || errno == ERANGE ||
      value < (long long)std::numeric_limits<T>::min() ||
      value > (long long)std::numeric_limits<T>::max()) {
    return false;
  }
  out = (T)value;
  return true;
}

struct Field {
  const char *name;
  bool (*set)(void *data, const std::string &value);
};

// 字段名即结构体成员表达式，值按成员类型解析
#define FIELD(T, member)                                                       \
  Field {                                                                      \
    #member, [](void *data, const std::string &value) {                       \
      return parse_value(value, static_cast<T *>(data)->member);              \
    }                                                                          \
  }

// USER_DATA的payload按文本填入
bool set_user_payload(void *data, const std::string &value) {
  auto *user_data = static_cast<UBDataUserData *>(data);
  if (value.size() > UB_USER_DATA_PAYLOAD_MAX) {
    return false;
  }
  user_data->payload_size = (uint8_t)value.size();
  std::memcpy(user_data->payload, value.data(), value.size());
  return true;
}

struct Msg {
  const char *name;
  ub_msg_id_t id;
  std::size_t size; // UBData*结构大小，无payload的消息为0
  std::vector<Field> fields;
};

// 全部下行消息
const std::vector<Msg> &messages() {
  static const std::vector<Msg> table = {
      {"restart",
       UB_MSG_RESTART,
       sizeof(UBDataRestart),
       {FIELD(UBDataRestart, delay),
        FIELD(UBDataRestart, only_restart_when_need)}},
      {"find", UB_MSG_FIND, sizeof(UBDataFind), {FIELD(UBDataFind, duration)}},
      {"read_param", UB_MSG_READ_PARAM, 0, {}},
      {"write_param",
       UB_MSG_WRITE_PARAM,
       sizeof(UBDataParam),
       {FIELD(UBDataParam, expect_z),
        FIELD(UBDataParam, z_noise),
        FIELD(UBDataParam, smooth_window),
        FIELD(UBDataParam, max_acceleration[0]),
        FIELD(UBDataParam, max_acceleration[1]),
        FIELD(UBDataParam, max_acceleration[2]),
        FIELD(UBDataParam, output.tag_pos),
        FIELD(UBDataParam, output.anchor_packet),
        FIELD(UBDataParam, output.anchor_pos),
        FIELD(UBDataParam, output.anchor_link_data),
        FIELD(UBDataParam, output.anchor_signal),
        FIELD(UBDataParam, output.anchor_ddoa),
        FIELD(UBDataParam, output.tag_pos_even_error),
        FIELD(UBDataParam, output.anchor_link_status),
        FIELD(UBDataParam, output.imu),
        FIELD(UBDataParam, output.attitude),
        FIELD(UBDataParam, output.dis),
        FIELD(UBDataParam, output.pdoa),
        FIELD(UBDataParam, sniff_duty_cycle),
        FIELD(UBDataParam, update_interval_max),
        FIELD(UBDataParam, reset_interval),
        FIELD(UBDataParam, min_sync_anchor_count),
        FIELD(UBDataParam, min_sniff_duty_cycle),
        FIELD(UBDataParam, disable_anchor_data_output_with_result),
        FIELD(UBDataParam, static_map_id)}},
      {"read_run_time_param", UB_MSG_READ_RUN_TIME_PARAM, 0, {}},
      {"write_run_time_param",
       UB_MSG_WRITE_RUN_TIME_PARAM,
       sizeof(UBDataRunTimeParam),
       {FIELD(UBDataRunTimeParam, sniff_duty_cycle)}},
      {"read_peripheral_config", UB_MSG_READ_PERIPHERAL_CONFIG, 0, {}},
      {"write_peripheral_config",
       UB_MSG_WRITE_PERIPHERAL_CONFIG,
       sizeof(UBDataPeripheralConfig),
       {FIELD(UBDataPeripheralConfig, uart),
        FIELD(UBDataPeripheralConfig, iic),
        FIELD(UBDataPeripheralConfig, uwb),
        FIELD(UBDataPeripheralConfig, ble),
        FIELD(UBDataPeripheralConfig, four_g),
        FIELD(UBDataPeripheralConfig, priority),
        FIELD(UBDataPeripheralConfig, motor),
        FIELD(UBDataPeripheralConfig, bee),
        FIELD(UBDataPeripheralConfig, key),
        FIELD(UBDataPeripheralConfig, motion),
        FIELD(UBDataPeripheralConfig, battery),
        FIELD(UBDataPeripheralConfig, battery_chargeable),
        FIELD(UBDataPeripheralConfig, baro),
        FIELD(UBDataPeripheralConfig, imu),
        FIELD(UBDataPeripheralConfig, gnss)}},
      {"read_hardware_peripheral_config",
       UB_MSG_READ_HARDWARE_PERIPHERAL_CONFIG,
       0,
       {}},
      {"read_final_peripheral_config",
       UB_MSG_READ_FINAL_PERIPHERAL_CONFIG,
       0,
       {}},
      {"read_uart_interface_param", UB_MSG_READ_UART_INTERFACE_PARAM, 0, {}},
      {"write_uart_interface_param",
       UB_MSG_WRITE_UART_INTERFACE_PARAM,
       sizeof(UBDataUartInterfaceParam),
       {FIELD(UBDataUartInterfaceParam, baudrate)}},
      {"read_iic_interface_param", UB_MSG_READ_IIC_INTERFACE_PARAM, 0, {}},
      {"write_iic_interface_param",
       UB_MSG_WRITE_IIC_INTERFACE_PARAM,
       sizeof(UBDataIicInterfaceParam),
       {FIELD(UBDataIicInterfaceParam, addr)}},
      {"read_uwb_interface_param", UB_MSG_READ_UWB_INTERFACE_PARAM, 0, {}},
      {"write_uwb_interface_param",
       UB_MSG_WRITE_UWB_INTERFACE_PARAM,
       sizeof(UBDataUwbInterfaceParam),
       {FIELD(UBDataUwbInterfaceParam, random_window_ms),
        FIELD(UBDataUwbInterfaceParam, max_rx_interval_ms),
        FIELD(UBDataUwbInterfaceParam, min_tx_interval_ms),
        FIELD(UBDataUwbInterfaceParam, dis_frequency),
        FIELD(UBDataUwbInterfaceParam, dis_slot_count_max),
        FIELD(UBDataUwbInterfaceParam, dis_remain_slot_count)}},
      {"user_data",
       UB_MSG_USER_DATA,
       sizeof(UBDataUserData),
       {Field{"payload", set_user_payload}}},
      {"z_measurement",
       UB_MSG_Z_MEASUREMENT,
       sizeof(UBDataZMeasurement),
       {FIELD(UBDataZMeasurement, z),
        FIELD(UBDataZMeasurement, z_std),
        FIELD(UBDataZMeasurement, timeout)}},
      {"state_control",
       UB_MSG_STATE_CONTROL,
       sizeof(UBDataStateControl),
       {FIELD(UBDataStateControl, sleep)}},
      {"map_measurement",
       UB_MSG_MAP_MEASUREMENT,
       sizeof(UBDataMapMeasurement),
       {FIELD(UBDataMapMeasurement, map_id),
        FIELD(UBDataMapMeasurement, timeout)}},
  };
  return table;
}

const Msg *find_by_name(const std::string &name) {
  for (const auto &msg : messages()) {
    if (name == msg.name) {
      return &msg;
    }
  }
  return nullptr;
}

const Msg *find_by_id(ub_msg_id_t id) {
  for (const auto &msg : messages()) {
    if (msg.id == id && msg.size != 0) {
      return &msg;
    }
  }
  return nullptr;
}

// 返回字段下标，不存在返回-1
int find_field(const Msg &msg, const std::string &name) {
  for (std::size_t i = 0; i < msg.fields.size(); i++) {
    if (name == msg.fields[i].name) {
      return (int)i;
    }
  }
  return -1;
}

std::string to_lower(std::string s) {
  for (auto &c : s) {
    c = (char)std::tolower((unsigned char)c);
  }
  return s;
}

// 设备最近回传的各消息结构体字节，由g_cache_mutex保护
std::mutex g_cache_mutex;
std::map<ub_msg_id_t, std::vector<uint8_t>> g_cache;

void print_help() {
  spdlog::info("commands: help | quit | exit | <message> [field=value ...]");
  spdlog::info("values: integer (decimal or 0x hex), float, bool (0/1/true/"
               "false); fields left out reuse the last value received from "
               "the device for that message, before that all fields are "
               "required");
  for (const auto &msg : messages()) {
    std::string fields;
    for (const auto &field : msg.fields) {
      fields += fmt::format(" {}", field.name);
    }
    spdlog::info("  {}{}", msg.name, fields);
  }
}

void send(const Msg &msg, const void *data) {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int frame_size =
      ub_prepare_msg_to_dev(msg.id, data, frame, sizeof(frame));
  if (frame_size <= 0) {
    spdlog::error("{}: prepare frame failed", msg.name);
    return;
  }
  serial_try_send_data(frame, frame_size);
  spdlog::info("{}: sent {} bytes: {:02X}", msg.name, frame_size,
               fmt::join(frame, frame + frame_size, ""));
}

} // namespace

namespace console_commands {

void remember(ub_msg_id_t msg_id, const void *msg_payload,
              int msg_payload_size) {
  const Msg *msg = find_by_id(msg_id);
  if (msg == nullptr || msg_payload == nullptr ||
      (std::size_t)msg_payload_size != msg->size) {
    return;
  }
  const auto *bytes = static_cast<const uint8_t *>(msg_payload);
  std::lock_guard<std::mutex> lock(g_cache_mutex);
  g_cache[msg_id].assign(bytes, bytes + msg_payload_size);
}

Result execute(const std::string &line) {
  std::istringstream iss(line);
  std::string name;
  iss >> name;
  name = to_lower(name);
  std::vector<std::string> args;
  for (std::string arg; iss >> arg;) {
    args.push_back(arg);
  }

  if (name == "quit" || name == "exit") {
    return Result::QUIT;
  }
  if (name == "help") {
    print_help();
    return Result::PAUSE_LOG;
  }
  const Msg *msg = find_by_name(name);
  if (msg == nullptr) {
    spdlog::error("unknown command '{}', type 'help'", name);
    return Result::CONTINUE;
  }
  if (msg->size == 0) {
    if (!args.empty()) {
      spdlog::error("{}: takes no fields", msg->name);
      return Result::CONTINUE;
    }
    send(*msg, nullptr);
    return Result::CONTINUE;
  }

  std::vector<uint8_t> data(msg->size, 0);
  bool has_base = false;
  {
    std::lock_guard<std::mutex> lock(g_cache_mutex);
    const auto it = g_cache.find(msg->id);
    if (it != g_cache.end()) {
      std::memcpy(data.data(), it->second.data(), msg->size);
      has_base = true;
    }
  }
  std::vector<bool> given(msg->fields.size(), false);
  for (const auto &arg : args) {
    const auto eq = arg.find('=');
    if (eq == std::string::npos) {
      spdlog::error("{}: expected field=value, got '{}'", msg->name, arg);
      return Result::CONTINUE;
    }
    const std::string key = arg.substr(0, eq);
    const std::string value = arg.substr(eq + 1);
    const int index = find_field(*msg, key);
    if (index < 0) {
      spdlog::error("{}: unknown field '{}'", msg->name, key);
      return Result::CONTINUE;
    }
    if (!msg->fields[index].set(data.data(), value)) {
      spdlog::error("{}: invalid value '{}' for field '{}'", msg->name, value,
                    key);
      return Result::CONTINUE;
    }
    given[index] = true;
  }
  if (!has_base) {
    std::vector<const char *> missing;
    for (std::size_t i = 0; i < given.size(); i++) {
      if (!given[i]) {
        missing.push_back(msg->fields[i].name);
      }
    }
    if (!missing.empty()) {
      spdlog::error("{}: nothing received from the device yet so every field "
                    "is required, missing: {}",
                    msg->name, fmt::join(missing, " "));
      return Result::CONTINUE;
    }
  }
  send(*msg, data.data());
  return Result::CONTINUE;
}

} // namespace console_commands
