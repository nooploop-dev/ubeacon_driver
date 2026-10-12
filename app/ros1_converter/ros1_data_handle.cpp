#include "ros1_data_handle.hpp"
#include "main_common.hpp"
#include <algorithm>
#include <cstring>

Ros1DataHandle::Ros1DataHandle(ros::NodeHandle *nh) : nh_(nh) {
  ub_parser_from_dev_init(&parser_, nullptr, &Ros1DataHandle::on_frame_msg,
                          nullptr, this);

  nh_->param<std::string>("frame_id", frame_id_, "map");
  ROS_INFO("Frame id: %s", frame_id_.c_str());

  // 设备 -> 用户(^)
  location_result_pub_ =
      nh_->advertise<ubeacon_driver::LocationResult>("location_result", 50);
  location_result_pose_pub_ =
      nh_->advertise<geometry_msgs::PoseStamped>("location_result_pose", 50);
  heartbeat_pub_ = nh_->advertise<ubeacon_driver::Heartbeat>("heartbeat", 50);
  anchor_signal_pub_ =
      nh_->advertise<ubeacon_driver::AnchorSignal>("anchor_signal", 50);
  anchor2tag_signal_pub_ = nh_->advertise<ubeacon_driver::Anchor2TagSignal>(
      "anchor2tag_signal", 50);
  tag2anchor_dis_pub_ =
      nh_->advertise<ubeacon_driver::Tag2AnchorDis>("tag2anchor_dis", 50);
  tag2anchor_pdoa_pub_ =
      nh_->advertise<ubeacon_driver::Tag2AnchorPdoa>("tag2anchor_pdoa", 50);
  imu_pub_ = nh_->advertise<ubeacon_driver::Imu>("imu", 50);
  attitude_pub_ = nh_->advertise<ubeacon_driver::Attitude>("attitude", 50);
  anchor2tag_ddoa_pub_ =
      nh_->advertise<ubeacon_driver::Anchor2TagDdoa>("anchor2tag_ddoa", 50);
  anchor_pos_pub_ =
      nh_->advertise<ubeacon_driver::AnchorPos>("anchor_pos", 50);
  global_time_status_pub_ = nh_->advertise<ubeacon_driver::GlobalTimeStatus>(
      "global_time_status", 50);
  user_data_from_device_pub_ =
      nh_->advertise<ubeacon_driver::UserData>("user_data_from_device", 50);
  param_pub_ = nh_->advertise<ubeacon_driver::Param>("param", 50);
  run_time_param_pub_ =
      nh_->advertise<ubeacon_driver::RunTimeParam>("run_time_param", 50);
  peripheral_config_pub_ = nh_->advertise<ubeacon_driver::PeripheralConfig>(
      "peripheral_config", 50);
  hardware_peripheral_config_pub_ =
      nh_->advertise<ubeacon_driver::PeripheralConfig>(
          "hardware_peripheral_config", 50);
  final_peripheral_config_pub_ =
      nh_->advertise<ubeacon_driver::PeripheralConfig>(
          "final_peripheral_config", 50);
  uart_interface_param_pub_ = nh_->advertise<ubeacon_driver::UartInterfaceParam>(
      "uart_interface_param", 50);
  iic_interface_param_pub_ = nh_->advertise<ubeacon_driver::IicInterfaceParam>(
      "iic_interface_param", 50);
  uwb_interface_param_pub_ = nh_->advertise<ubeacon_driver::UwbInterfaceParam>(
      "uwb_interface_param", 50);
  twr_param_pub_ = nh_->advertise<ubeacon_driver::TwrParam>("twr_param", 50);

  // 用户 -> 设备(v)
  find_sub_ = nh_->subscribe("find", 50, &Ros1DataHandle::on_find, this);
  restart_sub_ =
      nh_->subscribe("restart", 50, &Ros1DataHandle::on_restart, this);
  state_control_sub_ = nh_->subscribe(
      "state_control", 50, &Ros1DataHandle::on_state_control, this);
  z_measurement_sub_ = nh_->subscribe(
      "z_measurement", 50, &Ros1DataHandle::on_z_measurement, this);
  map_measurement_sub_ = nh_->subscribe(
      "map_measurement", 50, &Ros1DataHandle::on_map_measurement, this);
  user_data_to_device_sub_ = nh_->subscribe(
      "user_data_to_device", 50, &Ros1DataHandle::on_user_data_to_device, this);
  param_read_sub_ =
      nh_->subscribe("param_read", 50, &Ros1DataHandle::on_param_read, this);
  param_write_sub_ =
      nh_->subscribe("param_write", 50, &Ros1DataHandle::on_param_write, this);
  run_time_param_read_sub_ = nh_->subscribe(
      "run_time_param_read", 50, &Ros1DataHandle::on_run_time_param_read, this);
  run_time_param_write_sub_ =
      nh_->subscribe("run_time_param_write", 50,
                     &Ros1DataHandle::on_run_time_param_write, this);
  peripheral_config_read_sub_ =
      nh_->subscribe("peripheral_config_read", 50,
                     &Ros1DataHandle::on_peripheral_config_read, this);
  peripheral_config_write_sub_ =
      nh_->subscribe("peripheral_config_write", 50,
                     &Ros1DataHandle::on_peripheral_config_write, this);
  hardware_peripheral_config_read_sub_ =
      nh_->subscribe("hardware_peripheral_config_read", 50,
                     &Ros1DataHandle::on_hardware_peripheral_config_read, this);
  final_peripheral_config_read_sub_ =
      nh_->subscribe("final_peripheral_config_read", 50,
                     &Ros1DataHandle::on_final_peripheral_config_read, this);
  uart_interface_param_read_sub_ =
      nh_->subscribe("uart_interface_param_read", 50,
                     &Ros1DataHandle::on_uart_interface_param_read, this);
  iic_interface_param_read_sub_ =
      nh_->subscribe("iic_interface_param_read", 50,
                     &Ros1DataHandle::on_iic_interface_param_read, this);
  uwb_interface_param_read_sub_ =
      nh_->subscribe("uwb_interface_param_read", 50,
                     &Ros1DataHandle::on_uwb_interface_param_read, this);
  twr_param_read_sub_ = nh_->subscribe(
      "twr_param_read", 50, &Ros1DataHandle::on_twr_param_read, this);
}

// ---------------- 设备 -> 用户(^) ----------------

void Ros1DataHandle::dispatch(const UBDataLocationResult &data) {
  ubeacon_driver::LocationResult msg;
  msg.local_time_us = data.local_time_us;
  for (int i = 0; i < 3; i++) {
    msg.pos[i] = data.pos[i];
    msg.vel[i] = data.vel[i];
    msg.pos_noise[i] = data.pos_noise[i];
    msg.vel_noise[i] = data.vel_noise[i];
  }
  msg.map_id = data.map_id;
  msg.error_code = data.error_code;
  msg.area_id = data.area_id;
  location_result_pub_.publish(msg);

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = frame_id_;
  pose.pose.position.x = data.pos[0];
  pose.pose.position.y = data.pos[1];
  pose.pose.position.z = data.pos[2];
  pose.pose.orientation.w = 1.0;
  location_result_pose_pub_.publish(pose);
}

void Ros1DataHandle::dispatch(const UBDataHeartbeat &data) {
  ubeacon_driver::Heartbeat msg;
  msg.battery_percent = data.battery_percent;
  msg.battery_charging = data.battery_charging;
  msg.need_restart = data.need_restart;
  msg.reset_info_dirty = data.reset_info_dirty;
  msg.assert_info_dirty = data.assert_info_dirty;
  msg.restart_cnt = data.restart_cnt;
  msg.user_request_sleep = data.user_request_sleep;
  msg.hardware_enabled_motion = data.hardware_enabled_motion;
  msg.hardware_enabled_uart = data.hardware_enabled_uart;
  msg.hardware_enabled_iic = data.hardware_enabled_iic;
  msg.hardware_enabled_uwb = data.hardware_enabled_uwb;
  msg.hardware_enabled_ble = data.hardware_enabled_ble;
  msg.hardware_enabled_four_g = data.hardware_enabled_four_g;
  msg.hardware_enabled_baro = data.hardware_enabled_baro;
  msg.hardware_enabled_imu = data.hardware_enabled_imu;
  msg.hardware_enabled_gnss = data.hardware_enabled_gnss;
  msg.firmware_series = data.firmware_series;
  std::memcpy(msg.firmware_version.data(), data.firmware_version,
              sizeof(data.firmware_version));
  std::memcpy(msg.uid.data(), data.uid, UB_UID_SIZE);
  msg.battery_voltage = data.battery_voltage;
  msg.recent_rx_sts_error_occurred = data.recent_rx_sts_error_occurred;
  msg.recent_rx_data_error_occurred = data.recent_rx_data_error_occurred;
  msg.recent_global_time_error_occurred =
      data.recent_global_time_error_occurred;
  msg.recent_peer_global_time_error_occurred =
      data.recent_peer_global_time_error_occurred;
  msg.recent_peer_local_time_error_occurred =
      data.recent_peer_local_time_error_occurred;
  msg.rtos_warn = data.rtos_warn;
  msg.mcu_warn = data.mcu_warn;
  msg.uwb_warn = data.uwb_warn;
  heartbeat_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataAnchorSignal &data) {
  ubeacon_driver::AnchorSignal msg;
  msg.local_time_us = data.local_time_us;
  msg.area_id = data.area_id;
  msg.datas.resize(data.count);
  for (int i = 0; i < data.count; i++) {
    msg.datas[i].addr = data.datas[i].addr;
    msg.datas[i].fp_index = data.datas[i].fp_index;
    msg.datas[i].fp_to_peak = data.datas[i].fp_to_peak;
    msg.datas[i].mc = data.datas[i].mc;
    msg.datas[i].rx_rssi = data.datas[i].rx_rssi;
    msg.datas[i].fp_rssi = data.datas[i].fp_rssi;
    msg.datas[i].uwb_clock_offset_ppm = data.datas[i].uwb_clock_offset_ppm;
    msg.datas[i].mcu_clock_offset_ppm = data.datas[i].mcu_clock_offset_ppm;
    msg.datas[i].rx_rate = data.datas[i].rx_rate;
  }
  anchor_signal_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataAnchor2TagSignal &data) {
  using Msg = ubeacon_driver::Anchor2TagSignal;
  static_assert(Msg::LOCATION_MODE_TDOA ==
                    static_cast<uint8_t>(UB_LOCATION_MODE_TDOA),
                "LOCATION_MODE_TDOA mismatch");
  static_assert(Msg::LOCATION_MODE_TWR ==
                    static_cast<uint8_t>(UB_LOCATION_MODE_TWR),
                "LOCATION_MODE_TWR mismatch");
  static_assert(Msg::LOCATION_MODE_RSSI ==
                    static_cast<uint8_t>(UB_LOCATION_MODE_RSSI),
                "LOCATION_MODE_RSSI mismatch");
  Msg msg;
  msg.begin_time_us = data.begin_time_us;
  msg.location_mode = data.location_mode;
  msg.datas.resize(data.count);
  for (int i = 0; i < data.count; i++) {
    msg.datas[i].addr = data.datas[i].addr;
    msg.datas[i].rx_rssi = data.datas[i].rx_rssi;
    msg.datas[i].fp_rssi = data.datas[i].fp_rssi;
    msg.datas[i].uwb_clock_offset_ppm = data.datas[i].uwb_clock_offset_ppm;
    msg.datas[i].mcu_clock_offset_ppm = data.datas[i].mcu_clock_offset_ppm;
    msg.datas[i].rx_rate = data.datas[i].rx_rate;
  }
  anchor2tag_signal_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataTag2AnchorDis &data) {
  ubeacon_driver::Tag2AnchorDis msg;
  msg.begin_time_us = data.begin_time_us;
  msg.datas.resize(data.count);
  for (int i = 0; i < data.count; i++) {
    msg.datas[i].addr = data.datas[i].addr;
    msg.datas[i].dis = data.datas[i].dis;
  }
  tag2anchor_dis_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataTag2AnchorPdoa &data) {
  ubeacon_driver::Tag2AnchorPdoa msg;
  msg.begin_time_us = data.begin_time_us;
  msg.datas.resize(data.count);
  for (int i = 0; i < data.count; i++) {
    const auto &d = data.datas[i];
    msg.datas[i].addr = d.addr;
    msg.datas[i].antenna_type = d.antenna_type;
    msg.datas[i].pdoas.assign(d.pdoas, d.pdoas + d.pdoa_count);
  }
  tag2anchor_pdoa_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataAnchor2TagDdoa &data) {
  ubeacon_driver::Anchor2TagDdoa msg;
  msg.begin_time_us = data.begin_time_us;
  msg.group_count = data.group_count;
  msg.group_index = data.group_index;
  msg.datas.resize(data.count);
  for (int i = 0; i < data.count; i++) {
    msg.datas[i].a0 = data.datas[i].a0;
    msg.datas[i].a1 = data.datas[i].a1;
    msg.datas[i].ddoa = data.datas[i].ddoa;
  }
  anchor2tag_ddoa_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataAnchorPos &data) {
  ubeacon_driver::AnchorPos msg;
  msg.local_time_us = data.local_time_us;
  msg.src = data.src;
  for (int i = 0; i < 3; i++) {
    msg.pos[i] = data.pos[i];
  }
  msg.is_local_pos = data.is_local_pos;
  msg.map_id = data.map_id;
  msg.relative_map_z = data.relative_map_z;
  anchor_pos_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataGlobalTimeStatus &data) {
  ubeacon_driver::GlobalTimeStatus msg;
  msg.run = data.run;
  msg.timeout = data.timeout;
  msg.ttl = data.ttl;
  msg.time_us = data.time_us;
  msg.src = data.src;
  global_time_status_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataUserData &data) {
  ubeacon_driver::UserData msg;
  msg.payload.assign(data.payload, data.payload + data.payload_size);
  user_data_from_device_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataImu &data) {
  ubeacon_driver::Imu msg;
  msg.local_time_us = data.local_time_us;
  for (int i = 0; i < 3; i++) {
    msg.gyro[i] = data.gyro[i];
    msg.acc[i] = data.acc[i];
  }
  imu_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataAttitude &data) {
  ubeacon_driver::Attitude msg;
  msg.local_time_us = data.local_time_us;
  for (int i = 0; i < 4; i++) {
    msg.q[i] = data.q[i];
  }
  for (int i = 0; i < 3; i++) {
    msg.noise[i] = data.noise[i];
  }
  attitude_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataParam &data) {
  ubeacon_driver::Param msg;
  msg.expect_z = data.expect_z;
  msg.z_noise = data.z_noise;
  msg.smooth_window = data.smooth_window;
  for (int i = 0; i < 3; i++) {
    msg.max_acceleration[i] = data.max_acceleration[i];
  }
  msg.output_tag_pos = data.output.tag_pos;
  msg.output_anchor_packet = data.output.anchor_packet;
  msg.output_anchor_pos = data.output.anchor_pos;
  msg.output_anchor_link_data = data.output.anchor_link_data;
  msg.output_anchor_signal = data.output.anchor_signal;
  msg.output_anchor_ddoa = data.output.anchor_ddoa;
  msg.output_tag_pos_even_error = data.output.tag_pos_even_error;
  msg.output_anchor_link_status = data.output.anchor_link_status;
  msg.output_imu = data.output.imu;
  msg.output_attitude = data.output.attitude;
  msg.output_dis = data.output.dis;
  msg.output_pdoa = data.output.pdoa;
  msg.sniff_duty_cycle = data.sniff_duty_cycle;
  msg.update_interval_max = data.update_interval_max;
  msg.reset_interval = data.reset_interval;
  msg.min_sync_anchor_count = data.min_sync_anchor_count;
  msg.min_sniff_duty_cycle = data.min_sniff_duty_cycle;
  msg.static_map_id = data.static_map_id;
  param_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataRunTimeParam &data) {
  ubeacon_driver::RunTimeParam msg;
  msg.sniff_duty_cycle = data.sniff_duty_cycle;
  run_time_param_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(ub_msg_id_t msg_id,
                              const UBDataPeripheralConfig &data) {
  ubeacon_driver::PeripheralConfig msg;
  msg.uart = data.uart;
  msg.iic = data.iic;
  msg.uwb = data.uwb;
  msg.ble = data.ble;
  msg.four_g = data.four_g;
  msg.priority = data.priority;
  msg.motor = data.motor;
  msg.bee = data.bee;
  msg.key = data.key;
  msg.motion = data.motion;
  msg.battery = data.battery;
  msg.battery_chargeable = data.battery_chargeable;
  msg.baro = data.baro;
  msg.imu = data.imu;
  msg.gnss = data.gnss;
  switch (msg_id) {
  case UB_MSG_HARDWARE_PERIPHERAL_CONFIG:
    hardware_peripheral_config_pub_.publish(msg);
    break;
  case UB_MSG_FINAL_PERIPHERAL_CONFIG:
    final_peripheral_config_pub_.publish(msg);
    break;
  default:
    peripheral_config_pub_.publish(msg);
    break;
  }
}

void Ros1DataHandle::dispatch(const UBDataUartInterfaceParam &data) {
  ubeacon_driver::UartInterfaceParam msg;
  msg.baudrate = data.baudrate;
  uart_interface_param_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataIicInterfaceParam &data) {
  ubeacon_driver::IicInterfaceParam msg;
  msg.addr = data.addr;
  iic_interface_param_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataUwbInterfaceParam &data) {
  ubeacon_driver::UwbInterfaceParam msg;
  msg.random_window_ms = data.random_window_ms;
  msg.max_rx_interval_ms = data.max_rx_interval_ms;
  msg.min_tx_interval_ms = data.min_tx_interval_ms;
  uwb_interface_param_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataTwrParam &data) {
  ubeacon_driver::TwrParam msg;
  msg.dis_frequency = data.dis_frequency;
  msg.dis_slot_count_max = data.dis_slot_count_max;
  msg.dis_remain_slot_count = data.dis_remain_slot_count;
  twr_param_pub_.publish(msg);
}

// ---------------- 用户 -> 设备(v) ----------------

void Ros1DataHandle::on_find(const ubeacon_driver::Find::ConstPtr &msg) {
  UBDataFind data{};
  data.duration = msg->duration;
  main_common_send_msg(UB_MSG_FIND, &data, sizeof(data));
}

void Ros1DataHandle::on_restart(const ubeacon_driver::Restart::ConstPtr &msg) {
  UBDataRestart data{};
  data.delay = msg->delay;
  data.only_restart_when_need = msg->only_restart_when_need;
  main_common_send_msg(UB_MSG_RESTART, &data, sizeof(data));
}

void Ros1DataHandle::on_state_control(
    const ubeacon_driver::StateControl::ConstPtr &msg) {
  UBDataStateControl data{};
  data.sleep = msg->sleep;
  main_common_send_msg(UB_MSG_STATE_CONTROL, &data, sizeof(data));
}

void Ros1DataHandle::on_z_measurement(
    const ubeacon_driver::ZMeasurement::ConstPtr &msg) {
  UBDataZMeasurement data{};
  data.z = msg->z;
  data.z_std = msg->z_std;
  data.timeout = msg->timeout;
  main_common_send_msg(UB_MSG_Z_MEASUREMENT, &data, sizeof(data));
}

void Ros1DataHandle::on_map_measurement(
    const ubeacon_driver::MapMeasurement::ConstPtr &msg) {
  UBDataMapMeasurement data{};
  data.map_id = msg->map_id;
  data.timeout = msg->timeout;
  main_common_send_msg(UB_MSG_MAP_MEASUREMENT, &data, sizeof(data));
}

void Ros1DataHandle::on_user_data_to_device(
    const ubeacon_driver::UserData::ConstPtr &msg) {
  UBDataUserData data{};
  size_t n = std::min(msg->payload.size(), sizeof(data.payload));
  std::copy_n(msg->payload.begin(), n, data.payload);
  data.payload_size = (uint8_t)n;
  main_common_send_msg(UB_MSG_USER_DATA, &data, sizeof(data));
}

void Ros1DataHandle::on_param_read(const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_PARAM, nullptr, 0);
}

void Ros1DataHandle::on_param_write(const ubeacon_driver::Param::ConstPtr &msg) {
  UBDataParam data{};
  data.expect_z = msg->expect_z;
  data.z_noise = msg->z_noise;
  data.smooth_window = msg->smooth_window;
  for (int i = 0; i < 3; i++) {
    data.max_acceleration[i] = msg->max_acceleration[i];
  }
  data.output.tag_pos = msg->output_tag_pos;
  data.output.anchor_packet = msg->output_anchor_packet;
  data.output.anchor_pos = msg->output_anchor_pos;
  data.output.anchor_link_data = msg->output_anchor_link_data;
  data.output.anchor_signal = msg->output_anchor_signal;
  data.output.anchor_ddoa = msg->output_anchor_ddoa;
  data.output.tag_pos_even_error = msg->output_tag_pos_even_error;
  data.output.anchor_link_status = msg->output_anchor_link_status;
  data.output.imu = msg->output_imu;
  data.output.attitude = msg->output_attitude;
  data.output.dis = msg->output_dis;
  data.output.pdoa = msg->output_pdoa;
  data.sniff_duty_cycle = msg->sniff_duty_cycle;
  data.update_interval_max = msg->update_interval_max;
  data.reset_interval = msg->reset_interval;
  data.min_sync_anchor_count = msg->min_sync_anchor_count;
  data.min_sniff_duty_cycle = msg->min_sniff_duty_cycle;
  data.static_map_id = msg->static_map_id;
  main_common_send_msg(UB_MSG_WRITE_PARAM, &data, sizeof(data));
}

void Ros1DataHandle::on_run_time_param_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_RUN_TIME_PARAM, nullptr, 0);
}

void Ros1DataHandle::on_run_time_param_write(
    const ubeacon_driver::RunTimeParam::ConstPtr &msg) {
  UBDataRunTimeParam data{};
  data.sniff_duty_cycle = msg->sniff_duty_cycle;
  main_common_send_msg(UB_MSG_WRITE_RUN_TIME_PARAM, &data, sizeof(data));
}

void Ros1DataHandle::on_peripheral_config_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_PERIPHERAL_CONFIG, nullptr, 0);
}

void Ros1DataHandle::on_peripheral_config_write(
    const ubeacon_driver::PeripheralConfig::ConstPtr &msg) {
  UBDataPeripheralConfig data{};
  data.uart = msg->uart;
  data.iic = msg->iic;
  data.uwb = msg->uwb;
  data.ble = msg->ble;
  data.four_g = msg->four_g;
  data.priority = msg->priority;
  data.motor = msg->motor;
  data.bee = msg->bee;
  data.key = msg->key;
  data.motion = msg->motion;
  data.battery = msg->battery;
  data.battery_chargeable = msg->battery_chargeable;
  data.baro = msg->baro;
  data.imu = msg->imu;
  data.gnss = msg->gnss;
  main_common_send_msg(UB_MSG_WRITE_PERIPHERAL_CONFIG, &data, sizeof(data));
}

void Ros1DataHandle::on_hardware_peripheral_config_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_HARDWARE_PERIPHERAL_CONFIG, nullptr, 0);
}

void Ros1DataHandle::on_final_peripheral_config_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_FINAL_PERIPHERAL_CONFIG, nullptr, 0);
}

void Ros1DataHandle::on_uart_interface_param_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_UART_INTERFACE_PARAM, nullptr, 0);
}

void Ros1DataHandle::on_iic_interface_param_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_IIC_INTERFACE_PARAM, nullptr, 0);
}

void Ros1DataHandle::on_uwb_interface_param_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_UWB_INTERFACE_PARAM, nullptr, 0);
}

void Ros1DataHandle::on_twr_param_read(const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_TWR_PARAM, nullptr, 0);
}

// ---------------- 驱动回调 ----------------

void Ros1DataHandle::on_frame_msg(void *arg, ub_msg_id_t msg_id,
                                  const void *data, int data_size) {
  auto self = static_cast<Ros1DataHandle *>(arg);
  if (self == nullptr || data == nullptr || data_size == 0) {
    return;
  }
  switch (msg_id) {
  case UB_MSG_LOCATION_RESULT:
    self->dispatch(*static_cast<const UBDataLocationResult *>(data));
    break;
  case UB_MSG_HEARTBEAT:
    self->dispatch(*static_cast<const UBDataHeartbeat *>(data));
    break;
  case UB_MSG_ANCHOR_SIGNAL:
    self->dispatch(*static_cast<const UBDataAnchorSignal *>(data));
    break;
  case UB_MSG_ANCHOR2TAG_SIGNAL:
    self->dispatch(*static_cast<const UBDataAnchor2TagSignal *>(data));
    break;
  case UB_MSG_TAG2ANCHOR_DIS:
    self->dispatch(*static_cast<const UBDataTag2AnchorDis *>(data));
    break;
  case UB_MSG_TAG2ANCHOR_PDOA:
    self->dispatch(*static_cast<const UBDataTag2AnchorPdoa *>(data));
    break;
  case UB_MSG_IMU:
    self->dispatch(*static_cast<const UBDataImu *>(data));
    break;
  case UB_MSG_ATTITUDE:
    self->dispatch(*static_cast<const UBDataAttitude *>(data));
    break;
  case UB_MSG_ANCHOR2TAG_DDOA:
    self->dispatch(*static_cast<const UBDataAnchor2TagDdoa *>(data));
    break;
  case UB_MSG_ANCHOR_POS:
    self->dispatch(*static_cast<const UBDataAnchorPos *>(data));
    break;
  case UB_MSG_GLOBAL_TIME_STATUS:
    self->dispatch(*static_cast<const UBDataGlobalTimeStatus *>(data));
    break;
  case UB_MSG_USER_DATA:
    self->dispatch(*static_cast<const UBDataUserData *>(data));
    break;
  case UB_MSG_PARAM:
    self->dispatch(*static_cast<const UBDataParam *>(data));
    break;
  case UB_MSG_RUN_TIME_PARAM:
    self->dispatch(*static_cast<const UBDataRunTimeParam *>(data));
    break;
  case UB_MSG_PERIPHERAL_CONFIG:
  case UB_MSG_HARDWARE_PERIPHERAL_CONFIG:
  case UB_MSG_FINAL_PERIPHERAL_CONFIG:
    self->dispatch(msg_id,
                   *static_cast<const UBDataPeripheralConfig *>(data));
    break;
  case UB_MSG_UART_INTERFACE_PARAM:
    self->dispatch(*static_cast<const UBDataUartInterfaceParam *>(data));
    break;
  case UB_MSG_IIC_INTERFACE_PARAM:
    self->dispatch(*static_cast<const UBDataIicInterfaceParam *>(data));
    break;
  case UB_MSG_UWB_INTERFACE_PARAM:
    self->dispatch(*static_cast<const UBDataUwbInterfaceParam *>(data));
    break;
  case UB_MSG_TWR_PARAM:
    self->dispatch(*static_cast<const UBDataTwrParam *>(data));
    break;
  default:
    break;
  }
}
