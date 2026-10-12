#include "test_common.hpp"

#include <catch2/catch_approx.hpp>
#include <cmath>

using Catch::Approx;

// ============================================================
// 1. 全部消息的双向字节级 round-trip
//    等价于用 for_dev 侧充当设备仿真器，无需真实硬件即可证明编解码自洽
// ============================================================

TEST_CASE("round trip: restart") {
  UBDataRestart d{};
  d.delay = 3;
  d.only_restart_when_need = true;
  roundtrip(UB_MSG_RESTART, d);
}

TEST_CASE("round trip: find") {
  UBDataFind d{};
  d.duration = 10;
  roundtrip(UB_MSG_FIND, d);
}

TEST_CASE("round trip: param") {
  UBDataParam d{};
  d.expect_z = 1.2f;
  d.z_noise = 0.1f;
  d.smooth_window = 2;
  d.max_acceleration[0] = 0.6f;
  d.max_acceleration[1] = 0.6f;
  d.max_acceleration[2] = 0.02f;
  d.output.tag_pos = true;
  d.output.anchor_signal = true;
  d.output.anchor_ddoa = true;
  d.output.anchor_link_status = true;
  d.output.imu = true;
  d.output.attitude = true;
  d.output.dis = true;
  d.output.pdoa = true;
  d.sniff_duty_cycle = 20;
  d.update_interval_max = 5;
  d.reset_interval = 3000;
  d.min_sync_anchor_count = 2;
  d.min_sniff_duty_cycle = 5;
  d.static_map_id = 3;
  roundtrip(UB_MSG_PARAM, d);
}

TEST_CASE("round trip: run_time_param") {
  UBDataRunTimeParam d{};
  d.sniff_duty_cycle = 50;
  roundtrip(UB_MSG_RUN_TIME_PARAM, d);
}

TEST_CASE("round trip: peripheral_config") {
  UBDataPeripheralConfig d{};
  d.uart = true;
  d.uwb = true;
  d.four_g = true;
  d.priority = true;
  d.bee = true;
  d.key = true;
  d.battery = true;
  d.battery_chargeable = true;
  d.imu = true;
  d.gnss = true;
  roundtrip(UB_MSG_PERIPHERAL_CONFIG, d);
  // 硬件配置与最终配置只读，共用同一结构，仅上行
  roundtrip_to_user(UB_MSG_HARDWARE_PERIPHERAL_CONFIG, d);
  roundtrip_to_user(UB_MSG_FINAL_PERIPHERAL_CONFIG, d);
}

TEST_CASE("round trip: uart_interface_param") {
  UBDataUartInterfaceParam d{};
  d.baudrate = 115200;
  roundtrip(UB_MSG_UART_INTERFACE_PARAM, d);
}

TEST_CASE("round trip: iic_interface_param") {
  UBDataIicInterfaceParam d{};
  d.addr = 0x08;
  roundtrip(UB_MSG_IIC_INTERFACE_PARAM, d);
}

TEST_CASE("round trip: uwb_interface_param") {
  UBDataUwbInterfaceParam d{};
  d.random_window_ms = 20;
  d.max_rx_interval_ms = 1000;
  d.min_tx_interval_ms = 100;
  roundtrip(UB_MSG_UWB_INTERFACE_PARAM, d);
}

TEST_CASE("round trip: twr_param") {
  UBDataTwrParam d{};
  d.dis_frequency = 5.0f;
  d.dis_slot_count_max = 6;
  d.dis_remain_slot_count = 1;
  roundtrip(UB_MSG_TWR_PARAM, d);
}

TEST_CASE("round trip: location_result") {
  UBDataLocationResult d{};
  d.local_time_us = 123456789;
  d.pos[0] = 1.5f;
  d.pos[1] = -2.25f;
  d.pos[2] = 1.2f;
  d.vel[0] = 0.5f;
  d.vel[1] = -0.25f;
  d.vel[2] = 0.1f;
  d.pos_noise[0] = 0.1f;
  d.pos_noise[1] = 0.2f;
  d.pos_noise[2] = 0.3f;
  d.vel_noise[0] = 0.05f;
  d.vel_noise[1] = 0.05f;
  d.vel_noise[2] = 0.05f;
  d.map_id = 3;
  d.error_code = 0;
  d.area_id = 2;
  roundtrip_to_user(UB_MSG_LOCATION_RESULT, d);
}

TEST_CASE("round trip: heartbeat") {
  UBDataHeartbeat d{};
  d.battery_percent = 80;
  d.battery_charging = true;
  d.need_restart = false;
  d.reset_info_dirty = true;
  d.restart_cnt = 5;
  d.user_request_sleep = true;
  d.hardware_enabled_motion = true;
  d.hardware_enabled_uart = true;
  d.hardware_enabled_uwb = true;
  d.hardware_enabled_imu = true;
  d.firmware_series = 1;
  d.firmware_version[0] = 1;
  d.firmware_version[1] = 2;
  d.firmware_version[2] = 3;
  d.firmware_version[3] = 4;
  d.hardware_enabled_four_g = true;
  std::memcpy(d.uid, kUid, UB_UID_SIZE);
  d.battery_voltage = 4.0f;
  d.recent_rx_sts_error_occurred = true;
  d.recent_global_time_error_occurred = true;
  d.recent_peer_local_time_error_occurred = true;
  d.mcu_warn = true;
  roundtrip_to_user(UB_MSG_HEARTBEAT, d);
}

TEST_CASE("round trip: user_data") {
  UBDataUserData d{};
  d.payload_size = 5;
  for (uint8_t i = 0; i < d.payload_size; ++i) {
    d.payload[i] = static_cast<uint8_t>(i + 1);
  }
  roundtrip(UB_MSG_USER_DATA, d);
}

TEST_CASE("round trip: anchor_signal") {
  UBDataAnchorSignal d{};
  d.local_time_us = 987654321;
  d.count = 2;
  d.area_id = 1;
  d.datas[0].addr = 0x2001;
  d.datas[0].fp_index = 745.6f;
  d.datas[0].fp_to_peak = -5;
  d.datas[0].mc = 0.85f;
  d.datas[0].rx_rssi = -85.5f;
  d.datas[0].fp_rssi = -90.0f;
  d.datas[0].uwb_clock_offset_ppm = 1.23f;
  d.datas[0].mcu_clock_offset_ppm = -4.56f;
  d.datas[0].rx_rate = 1.0f;
  d.datas[1].addr = 0x2002;
  d.datas[1].fp_index = 750.0f;
  d.datas[1].fp_to_peak = 3;
  d.datas[1].mc = 0.5f;
  d.datas[1].rx_rssi = -95.0f;
  d.datas[1].fp_rssi = -99.5f;
  d.datas[1].uwb_clock_offset_ppm = -2.0f;
  d.datas[1].mcu_clock_offset_ppm = 3.0f;
  d.datas[1].rx_rate = 0.75f;
  roundtrip_to_user(UB_MSG_ANCHOR_SIGNAL, d);
}

TEST_CASE("round trip: anchor2tag_ddoa") {
  UBDataAnchor2TagDdoa d{};
  d.begin_time_us = 555;
  d.group_count = 2;
  d.group_index = 1;
  d.count = 2;
  d.datas[0].a0 = 0x3001;
  d.datas[0].a1 = 0x3002;
  d.datas[0].ddoa = 1.23f;
  d.datas[1].a0 = 0x3003;
  d.datas[1].a1 = 0x3004;
  d.datas[1].ddoa = -4.56f;
  roundtrip_to_user(UB_MSG_ANCHOR2TAG_DDOA, d);
}

TEST_CASE("round trip: anchor2tag_signal") {
  UBDataAnchor2TagSignal d{};
  d.begin_time_us = 111222333;
  d.count = 2;
  d.location_mode = UB_LOCATION_MODE_TWR;
  d.datas[0].addr = 0x2001;
  d.datas[0].rx_rssi = -85.5f;
  d.datas[0].fp_rssi = -90.0f;
  d.datas[0].uwb_clock_offset_ppm = 1.0f;
  d.datas[0].mcu_clock_offset_ppm = -5.0f;
  d.datas[0].rx_rate = 1.0f;
  d.datas[1].addr = 0x2002;
  d.datas[1].rx_rssi = -95.0f;
  d.datas[1].fp_rssi = -99.5f;
  d.datas[1].uwb_clock_offset_ppm = NAN;
  d.datas[1].mcu_clock_offset_ppm = 127.0f;
  d.datas[1].rx_rate = 0.75f;
  roundtrip_to_user(UB_MSG_ANCHOR2TAG_SIGNAL, d);
}

TEST_CASE("round trip: tag2anchor_dis") {
  UBDataTag2AnchorDis d{};
  d.begin_time_us = 444555666;
  d.count = 3;
  d.datas[0].addr = 0x2001;
  d.datas[0].dis = 1.23f;
  d.datas[1].addr = 0x2002;
  d.datas[1].dis = 12.34f;
  d.datas[2].addr = 0x2003;
  d.datas[2].dis = -0.05f;
  roundtrip_to_user(UB_MSG_TAG2ANCHOR_DIS, d);
}

TEST_CASE("round trip: tag2anchor_pdoa") {
  UBDataTag2AnchorPdoa d{};
  d.begin_time_us = 777888999;
  d.count = 2;
  d.datas[0].addr = 0x2001;
  d.datas[0].antenna_type = 1;
  d.datas[0].pdoa_count = 2;
  d.datas[0].pdoas[0] = 1.5f;
  d.datas[0].pdoas[1] = -0.75f;
  d.datas[1].addr = 0x2002;
  d.datas[1].antenna_type = 0;
  d.datas[1].pdoa_count = 0;
  roundtrip_to_user(UB_MSG_TAG2ANCHOR_PDOA, d);
}

TEST_CASE("round trip: imu") {
  UBDataImu d{};
  d.local_time_us = 123;
  d.gyro[0] = 0.01f;
  d.gyro[1] = -0.02f;
  d.gyro[2] = 0.03f;
  d.acc[0] = 0.1f;
  d.acc[1] = -0.2f;
  d.acc[2] = 9.8f;
  roundtrip_to_user(UB_MSG_IMU, d);
}

TEST_CASE("round trip: attitude") {
  UBDataAttitude d{};
  d.local_time_us = 456;
  d.q[0] = 1.0f;
  d.q[1] = 0.0f;
  d.q[2] = 0.0f;
  d.q[3] = 0.0f;
  d.noise[0] = 0.01f;
  d.noise[1] = 0.02f;
  d.noise[2] = 0.03f;
  roundtrip_to_user(UB_MSG_ATTITUDE, d);
}

TEST_CASE("round trip: z_measurement") {
  UBDataZMeasurement d{};
  d.z = 1.75f;
  d.z_std = 0.1f;
  d.timeout = 30;
  roundtrip(UB_MSG_Z_MEASUREMENT, d);
}

TEST_CASE("round trip: state_control") {
  UBDataStateControl d{};
  d.sleep = true;
  roundtrip(UB_MSG_STATE_CONTROL, d);
}

TEST_CASE("round trip: map_measurement") {
  UBDataMapMeasurement d{};
  d.map_id = 2;
  d.timeout = 60;
  roundtrip(UB_MSG_MAP_MEASUREMENT, d);
}

TEST_CASE("round trip: global_time_status") {
  UBDataGlobalTimeStatus d{};
  d.run = true;
  d.timeout = 10;
  d.ttl = 3;
  d.time_us = 1234567890123ULL; // 56 位以内
  d.src = 0x0001;
  roundtrip_to_user(UB_MSG_GLOBAL_TIME_STATUS, d);
}

TEST_CASE("round trip: anchor_pos") {
  UBDataAnchorPos d{};
  d.local_time_us = 424242;
  d.src = 0x4001;
  d.pos[0] = 1.0f;
  d.pos[1] = 2.0f;
  d.pos[2] = 3.0f;
  d.is_local_pos = true;
  d.map_id = 1;
  d.relative_map_z = -0.5f;
  roundtrip_to_user(UB_MSG_ANCHOR_POS, d);
}

// ============================================================
// 2. 定标正确性
//    仅有 round-trip 不足以证明定标系数正确（一对互逆但都错的系数也能通过），
//    因此这里用已知的原始字节直接钉住物理量。
// ============================================================

TEST_CASE("scaling: heartbeat battery_voltage = _battery_voltage / 40") {
  // 心跳线格式共 16 字节，[14] 是 _battery_voltage
  std::vector<uint8_t> payload(16, 0);
  payload[0] = 80;   // battery_percent=80, charging=0
  payload[14] = 160; // 160 / 40 = 4.0V
  auto cap = parse_from_dev(make_up_frame(UB_MSG_HEARTBEAT, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataHeartbeat *>(cap.msgs[0].data.data());
  REQUIRE(d.battery_percent == 80);
  REQUIRE(d.battery_charging == false);
  REQUIRE(d.battery_voltage == Approx(4.0f));
}

TEST_CASE("scaling: location_result field positions") {
  // 线格式 34 字节：
  //   [0..7] local_time  [8..19] pos(float)  [20..25] vel_cm(int16)
  //   [26..28] pos_noise_cm  [29..31] vel_noise_cm  [32] map_id
  //   [33] error_code:4 | area_id:4
  std::vector<uint8_t> payload(34, 0);
  payload[20] = 0x32; // vel[0] = 50cm/s
  payload[26] = 10;   // pos_noise[0] = 0.1m
  payload[32] = 3;    // map_id
  payload[33] = 0x21; // error_code = 1, area_id = 2
  auto cap = parse_from_dev(make_up_frame(UB_MSG_LOCATION_RESULT, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataLocationResult *>(cap.msgs[0].data.data());
  REQUIRE(d.vel[0] == Approx(0.5f));
  REQUIRE(d.pos_noise[0] == Approx(0.1f));
  REQUIRE(d.map_id == 3);
  REQUIRE(d.error_code == 1);
  REQUIRE(d.area_id == 2);

  // 上行组包同样是定长 34 字节：sof(1) size(2) uid(6) frame_id(1) id(1) 之后
  // 是消息的 payload_size
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP,
                                       UB_MSG_LOCATION_RESULT, &d, frame,
                                       sizeof(frame));
  REQUIRE(n > 0);
  REQUIRE((frame[11] & 0x7F) == 34);
}

TEST_CASE("scaling: heartbeat flag bit positions") {
  std::vector<uint8_t> payload(16, 0);
  payload[1] = 0xC0;  // bit6 user_request_sleep, bit7 hardware_enabled_motion
  payload[2] = 0xE0;  // bit5..7 hardware_enabled_baro / imu / gnss
  payload[15] = 0xE0; // bit5..7 rtos_warn / mcu_warn / uwb_warn
  auto cap = parse_from_dev(make_up_frame(UB_MSG_HEARTBEAT, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataHeartbeat *>(cap.msgs[0].data.data());
  REQUIRE(d.restart_cnt == 0);
  REQUIRE(d.user_request_sleep == true);
  REQUIRE(d.hardware_enabled_motion == true);
  REQUIRE(d.hardware_enabled_uart == false);
  REQUIRE(d.hardware_enabled_four_g == false);
  REQUIRE(d.hardware_enabled_baro == true);
  REQUIRE(d.hardware_enabled_imu == true);
  REQUIRE(d.hardware_enabled_gnss == true);
  REQUIRE(d.recent_peer_local_time_error_occurred == false);
  REQUIRE(d.rtos_warn == true);
  REQUIRE(d.mcu_warn == true);
  REQUIRE(d.uwb_warn == true);
}

TEST_CASE("scaling: anchor2tag_signal location_mode and int8 ppm") {
  // 头部：[0..7] begin_time  [8] reserved  [9] count:4 | location_mode:4
  // 每个元素 7 字节：addr(2) _rx_rssi _fp_rssi uwb_ppm(int8) mcu_ppm(int8)
  // _rx_rate
  std::vector<uint8_t> payload = {
      0, 0, 0, 0, 0, 0, 0, 0, 0,
      0x12,                                 // count = 2, location_mode = TWR
      0x01, 0x20, 170, 180, 0x80, 127, 255, // uwb_ppm = -128 即无效
      0x02, 0x20, 0, 0, 0x81, 0xFE, 0,      // -127, -2
  };
  auto cap = parse_from_dev(make_up_frame(UB_MSG_ANCHOR2TAG_SIGNAL, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d = *reinterpret_cast<const UBDataAnchor2TagSignal *>(
      cap.msgs[0].data.data());
  REQUIRE(d.count == 2);
  REQUIRE(d.location_mode == UB_LOCATION_MODE_TWR);
  REQUIRE(d.datas[0].addr == 0x2001);
  REQUIRE(d.datas[0].rx_rssi == Approx(-85.0f));
  REQUIRE(d.datas[0].fp_rssi == Approx(-90.0f));
  REQUIRE(std::isnan(d.datas[0].uwb_clock_offset_ppm));
  REQUIRE(d.datas[0].mcu_clock_offset_ppm == Approx(127.0f));
  REQUIRE(d.datas[0].rx_rate == Approx(1.0f));
  REQUIRE(d.datas[1].uwb_clock_offset_ppm == Approx(-127.0f));
  REQUIRE(d.datas[1].mcu_clock_offset_ppm == Approx(-2.0f));
}

TEST_CASE("anchor2tag_signal ppm encoding rounds, saturates and maps NaN") {
  UBDataAnchor2TagSignal d{};
  d.count = 2;
  d.datas[0].uwb_clock_offset_ppm = 300.0f;
  d.datas[0].mcu_clock_offset_ppm = -300.0f;
  d.datas[1].uwb_clock_offset_ppm = NAN;
  d.datas[1].mcu_clock_offset_ppm = 1.6f;
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP,
                                       UB_MSG_ANCHOR2TAG_SIGNAL, &d, frame,
                                       sizeof(frame));
  REQUIRE(n > 0);
  // 消息 payload 从 frame[12] 开始，元素从其中第 10 字节开始，ppm 在元素内
  // 第 4、5 字节
  const auto *e0 = reinterpret_cast<const int8_t *>(frame + 12 + 10);
  const auto *e1 = e0 + 7;
  REQUIRE(e0[4] == 127);
  REQUIRE(e0[5] == -127); // 饱和到 -127 而非 -128，-128 是无效值
  REQUIRE(e1[4] == -128);
  REQUIRE(e1[5] == 2);
}

TEST_CASE("legacy anchor2tag_signal id 89 is not recognized") {
  // 2.15.0.11 之前的固件用 89，且期间布局多次变化，驱动不再解析
  auto cap = parse_from_dev(make_up_frame(89, std::vector<uint8_t>(10, 0)));
  REQUIRE(cap.begin_count == 1);
  REQUIRE(cap.msgs.empty());
}

TEST_CASE("scaling: param z_noise / max_acceleration") {
  std::vector<uint8_t> payload(29, 0);
  payload[8] = 10;  // _z_noise -> 10 * 0.01 = 0.1m
  payload[9] = 2;   // smooth_window = 2
  payload[10] = 30; // _max_acceleration[0] -> 30 * 0.02 = 0.6
  payload[11] = 30;
  payload[12] = 1; // -> 1 * 0.02 = 0.02
  payload[26] = 3;    // static_map_id
  payload[28] = 0x0A; // output_attitude | output_pdoa
  auto cap = parse_from_dev(make_up_frame(UB_MSG_PARAM, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataParam *>(cap.msgs[0].data.data());
  REQUIRE(d.z_noise == Approx(0.1f));
  REQUIRE(d.smooth_window == 2);
  REQUIRE(d.max_acceleration[0] == Approx(0.6f));
  REQUIRE(d.max_acceleration[2] == Approx(0.02f));
  REQUIRE(d.static_map_id == 3);
  REQUIRE(d.output.imu == false);
  REQUIRE(d.output.attitude == true);
  REQUIRE(d.output.dis == false);
  REQUIRE(d.output.pdoa == true);
}

TEST_CASE("param write fills hidden wire fields with firmware defaults") {
  // 未公开的字段不能发 0：固件会把 expect_location_frequency=0 钳成 0.1Hz、
  // 把 locate_with_all_maps=0 当作关闭。线格式：[0..3] expect_location_frequency，
  // [25] bit0 locate_with_all_maps
  UBDataParam d{};
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n =
      ub_prepare_msg_to_dev(UB_MSG_WRITE_PARAM, &d, frame, sizeof(frame));
  REQUIRE(n > 0);
  // sof(1) + size(2) + frame_id(1) + msg id(1) + msg size(1) 之后是 payload
  const uint8_t *payload = frame + 6;
  float expect_location_frequency = 0;
  std::memcpy(&expect_location_frequency, payload, sizeof(float));
  REQUIRE(expect_location_frequency == Approx(1.0f));
  REQUIRE((payload[25] & 0x01) == 0x01);
  // 已弃用的 [25] bit1 与 output_delay_ms([27]) 写 0
  REQUIRE((payload[25] & 0x02) == 0);
  REQUIRE(payload[27] == 0);
}

TEST_CASE("scaling: uwb_interface_param field positions") {
  // 线格式 9 字节：
  //   [0..1] reserved  [2] _max_rx_interval(50ms)  [3..4] min_tx_interval_ms
  //   [5..6] reserved  [7..8] random_window_ms
  std::vector<uint8_t> payload(9, 0);
  payload[2] = 20;   // 20 * 50ms = 1.0s
  payload[3] = 100;  // 100ms
  payload[7] = 0x14; // 20ms
  auto cap = parse_from_dev(make_up_frame(UB_MSG_UWB_INTERFACE_PARAM, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d = *reinterpret_cast<const UBDataUwbInterfaceParam *>(
      cap.msgs[0].data.data());
  REQUIRE(d.max_rx_interval_ms == 1000);
  REQUIRE(d.min_tx_interval_ms == 100);
  REQUIRE(d.random_window_ms == 20);
}

TEST_CASE("scaling: twr_param field positions") {
  // 线格式 6 字节：
  //   [0..3] dis_frequency(float)  [4] dis_slot_count_max
  //   [5] dis_remain_slot_count
  std::vector<uint8_t> payload(6, 0);
  const float dis_frequency = 5.0f;
  std::memcpy(&payload[0], &dis_frequency, sizeof(dis_frequency));
  payload[4] = 6;
  payload[5] = 1;
  auto cap = parse_from_dev(make_up_frame(UB_MSG_TWR_PARAM, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataTwrParam *>(cap.msgs[0].data.data());
  REQUIRE(d.dis_frequency == Approx(5.0f));
  REQUIRE(d.dis_slot_count_max == 6);
  REQUIRE(d.dis_remain_slot_count == 1);
}

TEST_CASE("scaling: anchor2tag_ddoa group fields and ddoa_cm") {
  // 头部：[0..7] begin_time  [8] group_count:4 | group_index:4  [9] count
  // 每个元素 6 字节：a0(2) a1(2) ddoa_cm(int16)
  std::vector<uint8_t> payload = {
      0, 0, 0, 0, 0, 0, 0, 0,
      0x12, // group_count = 2, group_index = 1
      1,    // count
      0x01, 0x30, 0x02, 0x30, 0x85, 0xFF, // ddoa_cm = -123
  };
  auto cap = parse_from_dev(make_up_frame(UB_MSG_ANCHOR2TAG_DDOA, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d =
      *reinterpret_cast<const UBDataAnchor2TagDdoa *>(cap.msgs[0].data.data());
  REQUIRE(d.group_count == 2);
  REQUIRE(d.group_index == 1);
  REQUIRE(d.count == 1);
  REQUIRE(d.datas[0].a0 == 0x3001);
  REQUIRE(d.datas[0].a1 == 0x3002);
  REQUIRE(d.datas[0].ddoa == Approx(-1.23f));
}

// ============================================================
// 3. 帧层健壮性
// ============================================================

TEST_CASE("read request has empty payload") {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n =
      ub_prepare_msg_to_dev(UB_MSG_READ_PARAM, nullptr, frame, sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromUser parser;
  ub_parser_from_user_init(&parser, on_begin_from_user, on_msg, on_end, &cap);
  ub_parser_from_user_handle_data(&parser, frame, n);

  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == UB_MSG_READ_PARAM);
  REQUIRE(cap.msgs[0].data.empty());
}

TEST_CASE("parser handles byte-by-byte fragmentation") {
  UBDataFind d{};
  d.duration = 7;
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, UB_MSG_FIND,
                                       &d, frame,
                                       sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromDev parser;
  ub_parser_from_dev_init(&parser, on_begin_from_dev, on_msg, on_end, &cap);
  for (int i = 0; i < n; ++i) {
    ub_parser_from_dev_handle_data(&parser, &frame[i], 1);
  }

  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == UB_MSG_FIND);
}

TEST_CASE("parser resyncs past garbage bytes") {
  UBDataFind d{};
  d.duration = 7;
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n =
      ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, UB_MSG_FIND, &d,
                             frame, sizeof(frame));
  REQUIRE(n > 0);

  // 前置垃圾，且故意包含一个假的 SOF
  std::vector<uint8_t> bytes = {0x00, 0xFF, UB_FRAME_SOF, 0x12, 0x34};
  bytes.insert(bytes.end(), frame, frame + n);

  auto cap = parse_from_dev(bytes);
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == UB_MSG_FIND);
}

TEST_CASE("parser rejects bad checksum then recovers") {
  UBDataFind d{};
  d.duration = 7;
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n =
      ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, UB_MSG_FIND, &d,
                             frame, sizeof(frame));
  REQUIRE(n > 0);

  std::vector<uint8_t> bad(frame, frame + n);
  bad.back() ^= 0xFF; // 破坏校验和

  Capture cap;
  UBParserFromDev parser;
  ub_parser_from_dev_init(&parser, on_begin_from_dev, on_msg, on_end, &cap);
  ub_parser_from_dev_handle_data(&parser, bad.data(), static_cast<int>(bad.size()));
  REQUIRE(cap.msgs.empty()); // 坏帧被丢弃

  // 随后的好帧仍能被解析出来
  ub_parser_from_dev_handle_data(&parser, frame, n);
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == UB_MSG_FIND);
}

TEST_CASE("one frame carries multiple messages") {
  UBDataFind find{};
  find.duration = 7;
  UBDataStateControl sc{};
  sc.sleep = true;
  UBDataMapMeasurement mm{};
  mm.map_id = 2;
  mm.timeout = 60;

  uint8_t frame[UB_FRAME_SIZE_MAX];
  REQUIRE(ub_prepare_msg_to_dev_begin(frame, sizeof(frame)));
  REQUIRE(ub_prepare_msg_to_dev_try_append(UB_MSG_FIND, &find, frame,
                                           sizeof(frame)));
  REQUIRE(ub_prepare_msg_to_dev_try_append(UB_MSG_STATE_CONTROL, &sc, frame,
                                           sizeof(frame)));
  REQUIRE(ub_prepare_msg_to_dev_try_append(UB_MSG_MAP_MEASUREMENT, &mm, frame,
                                           sizeof(frame)));
  const int n = ub_prepare_msg_to_dev_end(frame, sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromUser parser;
  ub_parser_from_user_init(&parser, on_begin_from_user, on_msg, on_end, &cap);
  ub_parser_from_user_handle_data(&parser, frame, n);

  REQUIRE(cap.begin_count == 1); // 三条消息同属一帧
  REQUIRE(cap.end_count == 1);
  REQUIRE(cap.msgs.size() == 3);
  REQUIRE(cap.msgs[0].id == UB_MSG_FIND);
  REQUIRE(cap.msgs[1].id == UB_MSG_STATE_CONTROL);
  REQUIRE(cap.msgs[2].id == UB_MSG_MAP_MEASUREMENT);
}

// ============================================================
// 4. 变长消息
// ============================================================

TEST_CASE("variable length messages only send valid elements") {
  auto frame_size_for = [](uint8_t count) {
    UBDataTag2AnchorDis d{};
    d.count = count;
    uint8_t frame[UB_FRAME_SIZE_MAX];
    return ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP,
                                  UB_MSG_TAG2ANCHOR_DIS, &d, frame,
                                  sizeof(frame));
  };

  const int n0 = frame_size_for(0);
  const int n3 = frame_size_for(3);
  const int n9 = frame_size_for(9);
  REQUIRE(n0 > 0);
  // 每多一个信标，线上就多 4 字节（addr 2 + dis_cm 2）
  REQUIRE(n3 == n0 + 3 * 4);
  REQUIRE(n9 == n0 + 9 * 4);
}

TEST_CASE("anchor_signal at max count fits in the 7-bit payload limit") {
  // 满 9 个信标时线格式恰好 127 字节，正好压满 payload_size 的 7 位上限
  UBDataAnchorSignal d{};
  d.count = UB_ANCHOR_SIGNAL_DATA_MAX;
  for (int i = 0; i < UB_ANCHOR_SIGNAL_DATA_MAX; ++i) {
    d.datas[i].addr = static_cast<ub_addr_t>(0x2000 + i);
    d.datas[i].rx_rssi = -85.5f;
    d.datas[i].rx_rate = 1.0f;
  }
  roundtrip_to_user(UB_MSG_ANCHOR_SIGNAL, d);
}

TEST_CASE("out-of-range count from a corrupt device is clamped") {
  // count 是 4 位（最大 15），但数组只有 9 个元素；必须钳位而不是越界
  std::vector<uint8_t> payload(10 + UB_ANCHOR2TAG_SIGNAL_DATA_MAX * 7, 0);
  payload[9] = 0x0F; // count = 15, location_mode = TDOA
  auto cap = parse_from_dev(make_up_frame(UB_MSG_ANCHOR2TAG_SIGNAL, payload));

  REQUIRE(cap.msgs.size() == 1);
  const auto &d = *reinterpret_cast<const UBDataAnchor2TagSignal *>(
      cap.msgs[0].data.data());
  REQUIRE(d.count == UB_ANCHOR2TAG_SIGNAL_DATA_MAX);
}

// ============================================================
// 5. 未知消息
// ============================================================

TEST_CASE("unknown message id is ignored, frame still delimited") {
  auto cap = parse_from_dev(make_up_frame(200, {0x01, 0x02}));
  REQUIRE(cap.begin_count == 1);
  REQUIRE(cap.end_count == 1);
  REQUIRE(cap.msgs.empty()); // 未识别的消息不上抛
}

// ============================================================
// 6. frame_id：标明设备类型，由 on_frame_begin 决定是否处理该帧
// ============================================================

TEST_CASE("on_frame_begin receives the frame_id of the frame") {
  auto cap = parse_from_dev(make_up_frame(UB_MSG_FIND, {10}));
  REQUIRE(cap.begin_count == 1);
  REQUIRE(cap.frame_id == UB_FRAME_ID_TAG_UP);
  REQUIRE(cap.msgs.size() == 1);
}

TEST_CASE("frames from other device types are surfaced, not filtered") {
  // 驱动自身不过滤：gateway/anchor 的上行帧同样会回调 on_frame_begin，
  // 是否处理完全由回调决定(本例的 on_begin_from_dev 只认 tag)
  for (ub_frame_id_t id : {static_cast<ub_frame_id_t>(UB_FRAME_ID_GATEWAY_UP),
                           static_cast<ub_frame_id_t>(UB_FRAME_ID_ANCHOR_UP)}) {
    auto cap = parse_from_dev(make_up_frame(UB_MSG_FIND, {10}, id));
    REQUIRE(cap.begin_count == 1);
    REQUIRE(cap.frame_id == id);
    REQUIRE(cap.msgs.empty());
    REQUIRE(cap.end_count == 0); // 被拒的帧不回调 on_frame_end
  }
}

TEST_CASE("a callback accepting any uplink device type gets the messages") {
  const auto bytes =
      make_up_frame(UB_MSG_FIND, {10},
                    static_cast<ub_frame_id_t>(UB_FRAME_ID_ANCHOR_UP));
  Capture cap;
  UBParserFromDev parser;
  ub_parser_from_dev_init(
      &parser,
      [](void *arg, const uint8_t *uid, ub_frame_id_t frame_id) {
        auto *c = static_cast<Capture *>(arg);
        c->begin_count++;
        c->uid.assign(uid, uid + UB_UID_SIZE);
        c->frame_id = frame_id;
        return frame_id == UB_FRAME_ID_GATEWAY_UP ||
               frame_id == UB_FRAME_ID_TAG_UP ||
               frame_id == UB_FRAME_ID_ANCHOR_UP;
      },
      on_msg, on_end, &cap);
  ub_parser_from_dev_handle_data(&parser, bytes.data(),
                                 static_cast<int>(bytes.size()));
  REQUIRE(cap.frame_id == UB_FRAME_ID_ANCHOR_UP);
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.end_count == 1);
}

TEST_CASE("without on_frame_begin the driver does not filter by frame_id") {
  const auto bytes =
      make_up_frame(UB_MSG_FIND, {10},
                    static_cast<ub_frame_id_t>(UB_FRAME_ID_ANCHOR_UP));
  Capture cap;
  UBParserFromDev parser;
  ub_parser_from_dev_init(&parser, nullptr, on_msg, on_end, &cap);
  ub_parser_from_dev_handle_data(&parser, bytes.data(),
                                 static_cast<int>(bytes.size()));
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.end_count == 1);
}

TEST_CASE("uplink frames carry the frame_id the device was built with") {
  UBDataFind d{};
  d.duration = 10;
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_ANCHOR_UP, UB_MSG_FIND,
                                       &d, frame, sizeof(frame));
  REQUIRE(n > 0);
  // sof(1) + payload_size(2) + uid(6) 之后就是 frame_id
  REQUIRE(frame[3 + UB_UID_SIZE] == UB_FRAME_ID_ANCHOR_UP);
}

TEST_CASE("from_user parser passes frame_id and honours the veto") {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  UBDataFind d{};
  d.duration = 10;
  const int n = ub_prepare_msg_to_dev(UB_MSG_FIND, &d, frame, sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromUser parser;
  ub_parser_from_user_init(&parser, on_begin_from_user, on_msg, on_end, &cap);
  ub_parser_from_user_handle_data(&parser, frame, n);
  REQUIRE(cap.begin_count == 1);
  REQUIRE(cap.frame_id == UB_FRAME_ID_DOWN);
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.end_count == 1);
}

// ============================================================
// 7. 方向归属：msg_id 只在其标注的方向上有效
//    方向标注见 ubeacon_driver_data.h 中每个 UB_MSG_* 后的 v/^ 注释
// ============================================================

TEST_CASE("uplink-only messages cannot be built as a downlink frame") {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  UBDataLocationResult lr{};
  REQUIRE(ub_prepare_msg_to_dev(UB_MSG_LOCATION_RESULT, &lr, frame,
                                sizeof(frame)) < 0);
  UBDataHeartbeat hb{};
  REQUIRE(ub_prepare_msg_to_dev(UB_MSG_HEARTBEAT, &hb, frame, sizeof(frame)) <
          0);
  UBDataAnchorPos ap{};
  REQUIRE(ub_prepare_msg_to_dev(UB_MSG_ANCHOR_POS, &ap, frame, sizeof(frame)) <
          0);
}

TEST_CASE("downlink-only messages cannot be built as an uplink frame") {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  // READ_* 只在下行存在，上行构造应失败
  REQUIRE(ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, UB_MSG_READ_PARAM,
                                 nullptr, frame, sizeof(frame)) < 0);
}

TEST_CASE("hardware/final peripheral config are read-only") {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  UBDataPeripheralConfig d{};
  // 只读配置没有 WRITE 消息，下行只能发读请求
  REQUIRE(ub_prepare_msg_to_dev(UB_MSG_HARDWARE_PERIPHERAL_CONFIG, &d, frame,
                                sizeof(frame)) < 0);
  REQUIRE(ub_prepare_msg_to_dev(UB_MSG_FINAL_PERIPHERAL_CONFIG, &d, frame,
                                sizeof(frame)) < 0);
  REQUIRE(ub_prepare_msg_to_dev(UB_MSG_READ_HARDWARE_PERIPHERAL_CONFIG, nullptr,
                                frame, sizeof(frame)) > 0);
  REQUIRE(ub_prepare_msg_to_dev(UB_MSG_READ_FINAL_PERIPHERAL_CONFIG, nullptr,
                                frame, sizeof(frame)) > 0);
}

TEST_CASE("read requests are downlink-only, empty, and decoded as such") {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n =
      ub_prepare_msg_to_dev(UB_MSG_READ_PARAM, nullptr, frame, sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromUser parser;
  ub_parser_from_user_init(&parser, on_begin_from_user, on_msg, on_end, &cap);
  ub_parser_from_user_handle_data(&parser, frame, n);
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == UB_MSG_READ_PARAM);
  REQUIRE(cap.msgs[0].data.empty()); // 空消息，回调收到 data == NULL

  // 同一个 id 出现在上行帧里则不被识别，整条消息忽略
  auto up = parse_from_dev(make_up_frame(UB_MSG_READ_PARAM, {}));
  REQUIRE(up.begin_count == 1);
  REQUIRE(up.end_count == 1);
  REQUIRE(up.msgs.empty());
}

TEST_CASE("twr param read request is an empty downlink message") {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_dev(UB_MSG_READ_TWR_PARAM, nullptr, frame,
                                      sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromUser parser;
  ub_parser_from_user_init(&parser, on_begin_from_user, on_msg, on_end, &cap);
  ub_parser_from_user_handle_data(&parser, frame, n);
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == UB_MSG_READ_TWR_PARAM);
  REQUIRE(cap.msgs[0].data.empty());
}

TEST_CASE("reset_param is an empty message in both directions") {
  // 设备收到后原样回一条空消息作为确认
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_dev(UB_MSG_RESET_PARAM, nullptr, frame,
                                      sizeof(frame));
  REQUIRE(n > 0);
  Capture down;
  UBParserFromUser parser;
  ub_parser_from_user_init(&parser, on_begin_from_user, on_msg, on_end, &down);
  ub_parser_from_user_handle_data(&parser, frame, n);
  REQUIRE(down.msgs.size() == 1);
  REQUIRE(down.msgs[0].id == UB_MSG_RESET_PARAM);
  REQUIRE(down.msgs[0].data.empty());

  REQUIRE(ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, UB_MSG_RESET_PARAM,
                                 nullptr, frame, sizeof(frame)) > 0);
  // 固件按 sizeof(空结构体) 回包，payload 实际是 1 个无意义字节
  for (const auto &payload :
       {std::vector<uint8_t>{}, std::vector<uint8_t>{0}}) {
    auto up = parse_from_dev(make_up_frame(UB_MSG_RESET_PARAM, payload));
    REQUIRE(up.msgs.size() == 1);
    REQUIRE(up.msgs[0].id == UB_MSG_RESET_PARAM);
    REQUIRE(up.msgs[0].data.empty());
  }
}

TEST_CASE("uplink-only messages in a downlink frame are ignored") {
  // 设备侧收到一条上行才有的消息(如 HEARTBEAT)，应当忽略而不是误解码
  UBDataHeartbeat hb{};
  hb.battery_percent = 80;
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP,
                                       UB_MSG_HEARTBEAT, &hb, frame,
                                       sizeof(frame));
  REQUIRE(n > 0);

  // 把上行帧的 msg 部分原样塞进一个下行帧
  const int up_header = UB_UID_SIZE + 1;
  std::vector<uint8_t> msgs(frame + 3 + up_header, frame + n - 1);
  std::vector<uint8_t> down;
  down.push_back(UB_FRAME_SOF);
  const auto payload_size = static_cast<uint16_t>(1 + msgs.size());
  down.push_back(static_cast<uint8_t>(payload_size & 0xFF));
  down.push_back(static_cast<uint8_t>(payload_size >> 8));
  down.push_back(UB_FRAME_ID_DOWN);
  down.insert(down.end(), msgs.begin(), msgs.end());
  uint8_t sum = 0;
  for (uint8_t b : down) {
    sum += b;
  }
  down.push_back(sum);

  Capture cap;
  UBParserFromUser parser;
  ub_parser_from_user_init(&parser, on_begin_from_user, on_msg, on_end, &cap);
  ub_parser_from_user_handle_data(&parser, down.data(),
                                  static_cast<int>(down.size()));
  REQUIRE(cap.begin_count == 1);
  REQUIRE(cap.end_count == 1);
  REQUIRE(cap.msgs.empty()); // HEARTBEAT 在下行方向不存在
}

// ============================================================
// 8. 扩展消息：通过函数指针注入，可选且按方向分开
// ============================================================

namespace {

// 一条自定义的上行消息：线格式 2 字节，结构体是放大后的 int
constexpr ub_msg_id_t kExtMsgId = 200;
struct ExtData {
  int value;
};

void ext_encode_dev_to_user(ub_msg_id_t msg_id, const void *data, void *raw,
                            int *raw_size) {
  if (msg_id != kExtMsgId) {
    return; // 保持 *raw_size 为 -1，表示不认识
  }
  const auto v = static_cast<const ExtData *>(data)->value;
  auto *p = static_cast<uint8_t *>(raw);
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  *raw_size = 2;
}

int ext_decode_dev_to_user(ub_msg_id_t msg_id, const void *payload,
                           int payload_size, void *data_buf,
                           int data_buf_size) {
  if (msg_id != kExtMsgId || payload_size < 2 ||
      data_buf_size < static_cast<int>(sizeof(ExtData))) {
    return -1;
  }
  const auto *p = static_cast<const uint8_t *>(payload);
  static_cast<ExtData *>(data_buf)->value = p[0] | (p[1] << 8);
  return static_cast<int>(sizeof(ExtData));
}

// 注入的钩子是全局的，用 RAII 保证用例之间互不影响
struct ExtendGuard {
  ExtendGuard() {
    ub_set_encode_dev_to_user_extend(ext_encode_dev_to_user);
    ub_set_decode_dev_to_user_extend(ext_decode_dev_to_user);
  }
  ~ExtendGuard() {
    ub_set_encode_dev_to_user_extend(nullptr);
    ub_set_decode_dev_to_user_extend(nullptr);
  }
};

} // namespace

TEST_CASE("without injection an extend msg id is unknown in both directions") {
  ExtData d{0x1234};
  uint8_t frame[UB_FRAME_SIZE_MAX];
  REQUIRE(ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, kExtMsgId, &d, frame,
                                 sizeof(frame)) < 0);
  auto cap = parse_from_dev(make_up_frame(kExtMsgId, {0x34, 0x12}));
  REQUIRE(cap.msgs.empty());
}

TEST_CASE("injected hooks round-trip a custom message") {
  ExtendGuard guard;

  ExtData d{0x1234};
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, kExtMsgId, &d,
                                       frame, sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromDev parser;
  ub_parser_from_dev_init(&parser, on_begin_from_dev, on_msg, on_end, &cap);
  ub_parser_from_dev_handle_data(&parser, frame, n);

  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == kExtMsgId);
  REQUIRE(cap.msgs[0].data.size() == sizeof(ExtData));
  ExtData decoded{};
  std::memcpy(&decoded, cap.msgs[0].data.data(), sizeof(ExtData));
  REQUIRE(decoded.value == d.value);
}

TEST_CASE("injection is per direction") {
  ExtendGuard guard; // 只注入了 dev_to_user 两个钩子

  ExtData d{0x1234};
  uint8_t frame[UB_FRAME_SIZE_MAX];
  // 下行未注入，同一个 msg_id 仍然不被识别
  REQUIRE(ub_prepare_msg_to_dev(kExtMsgId, &d, frame, sizeof(frame)) < 0);
}

TEST_CASE("extend hooks cannot override built-in messages") {
  // 钩子只在内建表未命中时才被调用：内建消息仍走内建实现
  ub_set_encode_dev_to_user_extend(
      [](ub_msg_id_t, const void *, void *, int *raw_size) { *raw_size = 99; });
  UBDataFind find{};
  find.duration = 7;
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, UB_MSG_FIND,
                                       &find, frame, sizeof(frame));
  ub_set_encode_dev_to_user_extend(nullptr);

  REQUIRE(n > 0);
  auto cap = parse_from_dev(std::vector<uint8_t>(frame, frame + n));
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == UB_MSG_FIND);
  REQUIRE(cap.msgs[0].data.size() == sizeof(UBDataFind));
}
