// 版本兼容：新旧固件的 payload 长度不一致时，解析器必须安全降级
//
// 规则由 ub_msg_copy_payload 保证：payload 比结构短则补零，比结构长则截断。

#include "test_common.hpp"

#include <catch2/catch_approx.hpp>

using Catch::Approx;

// 心跳线格式共 16 字节：
//   [0] battery_percent:7 | battery_charging:1
//   [1] need_restart / reset_info_dirty / assert_info_dirty / restart_cnt:3 /
//       user_request_sleep / hardware_enabled_motion
//   [2] hardware_enabled_*
//   [3] firmware_series
//   [4..7]  firmware_version[4]
//   [8..13] uid[6]
//   [14] _battery_voltage
//   [15] recent_*_error_occurred / rtos_warn / mcu_warn / uwb_warn
static constexpr int kHeartbeatRawSize = 16;

TEST_CASE("old firmware: short payload is zero-filled") {
  // 只发前 4 个字节，尾部字段（版本号、uid、电压）应被补零而不是读到脏数据
  std::vector<uint8_t> payload = {
      0xD0, // battery_percent=80(0x50) | charging=1(0x80)
      0x00, 0x00,
      7, // firmware_series
  };
  auto cap = parse_from_dev(make_up_frame(UB_MSG_HEARTBEAT, payload));

  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].data.size() == sizeof(UBDataHeartbeat));
  const auto &d =
      *reinterpret_cast<const UBDataHeartbeat *>(cap.msgs[0].data.data());

  // 收到的字段正常解析
  REQUIRE(d.battery_percent == 80);
  REQUIRE(d.battery_charging == true);
  REQUIRE(d.firmware_series == 7);

  // 未收到的字段全部为零
  REQUIRE(d.battery_voltage == Approx(0.0f));
  for (int i = 0; i < 4; ++i) {
    REQUIRE(d.firmware_version[i] == 0);
  }
  for (int i = 0; i < UB_UID_SIZE; ++i) {
    REQUIRE(d.uid[i] == 0);
  }
}

TEST_CASE("new firmware: extra trailing bytes are truncated") {
  // 新固件在尾部追加了字段，旧驱动应忽略多余部分而不是解析失败
  std::vector<uint8_t> payload(kHeartbeatRawSize, 0);
  payload[0] = 80;
  payload[3] = 7;
  payload[14] = 160; // 4.0V
  std::vector<uint8_t> extended = payload;
  extended.insert(extended.end(), {0xAA, 0xBB, 0xCC, 0xDD}); // 未来新增字段

  auto cap = parse_from_dev(make_up_frame(UB_MSG_HEARTBEAT, extended));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataHeartbeat *>(cap.msgs[0].data.data());
  REQUIRE(d.battery_percent == 80);
  REQUIRE(d.firmware_series == 7);
  REQUIRE(d.battery_voltage == Approx(4.0f));
}

TEST_CASE("old firmware: 18-byte LocationParam decodes, new fields zeroed") {
  // 旧固件的 LocationParam 到 reset_interval 为止共 18 字节
  std::vector<uint8_t> payload(18, 0);
  payload[9] = 2;    // smooth_window
  payload[16] = 0xB8; // reset_interval = 3000
  payload[17] = 0x0B;
  auto cap = parse_from_dev(make_up_frame(UB_MSG_PARAM, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataParam *>(cap.msgs[0].data.data());
  REQUIRE(d.smooth_window == 2);
  REQUIRE(d.reset_interval == 3000);
  REQUIRE(d.static_map_id == 0);
  REQUIRE(d.output.attitude == false);
}

TEST_CASE("pre-2.15.0.9 LocationResult with trailing anchors is truncated") {
  // 旧固件在 34 字节之后还附带 reserved、anchor_count 与 anchors，驱动只取前 34 字节
  std::vector<uint8_t> payload(72, 0);
  payload[32] = 3;    // map_id
  payload[33] = 0x21; // error_code = 1, area_id = 2
  payload[35] = 1;    // 旧 anchor_count
  payload[36] = 0x01; // 旧 anchors[0].addr
  auto cap = parse_from_dev(make_up_frame(UB_MSG_LOCATION_RESULT, payload));

  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].data.size() == sizeof(UBDataLocationResult));
  const auto &d =
      *reinterpret_cast<const UBDataLocationResult *>(cap.msgs[0].data.data());
  REQUIRE(d.map_id == 3);
  REQUIRE(d.error_code == 1);
  REQUIRE(d.area_id == 2);
}

TEST_CASE("empty payload on a data message decodes to a zeroed struct") {
  // 极端情况：payload 完全为空，也不应崩溃或读到脏数据
  auto cap = parse_from_dev(make_up_frame(UB_MSG_HEARTBEAT, {}));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataHeartbeat *>(cap.msgs[0].data.data());
  REQUIRE(d.battery_percent == 0);
  REQUIRE(d.battery_voltage == Approx(0.0f));
}
