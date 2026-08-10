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
  anchor_ddoas_pub_ =
      nh_->advertise<ubeacon_driver::AnchorDdoas>("anchor_ddoas", 50);
  anchor_pos_pub_ =
      nh_->advertise<ubeacon_driver::AnchorPos>("anchor_pos", 50);
  global_time_status_pub_ = nh_->advertise<ubeacon_driver::GlobalTimeStatus>(
      "global_time_status", 50);
  user_data_from_device_pub_ =
      nh_->advertise<ubeacon_driver::UserData>("user_data_from_device", 50);
  param_pub_ = nh_->advertise<ubeacon_driver::Param>("param", 50);
  run_time_param_pub_ =
      nh_->advertise<ubeacon_driver::RunTimeParam>("run_time_param", 50);
  interface_param_pub_ =
      nh_->advertise<ubeacon_driver::InterfaceParam>("interface_param", 50);
  uart_interface_param_pub_ = nh_->advertise<ubeacon_driver::UartInterfaceParam>(
      "uart_interface_param", 50);
  iic_interface_param_pub_ = nh_->advertise<ubeacon_driver::IicInterfaceParam>(
      "iic_interface_param", 50);
  uwb_interface_param_pub_ = nh_->advertise<ubeacon_driver::UwbInterfaceParam>(
      "uwb_interface_param", 50);
  ble_interface_param_pub_ = nh_->advertise<ubeacon_driver::BleInterfaceParam>(
      "ble_interface_param", 50);

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
  interface_param_read_sub_ =
      nh_->subscribe("interface_param_read", 50,
                     &Ros1DataHandle::on_interface_param_read, this);
  uart_interface_param_read_sub_ =
      nh_->subscribe("uart_interface_param_read", 50,
                     &Ros1DataHandle::on_uart_interface_param_read, this);
  iic_interface_param_read_sub_ =
      nh_->subscribe("iic_interface_param_read", 50,
                     &Ros1DataHandle::on_iic_interface_param_read, this);
  uwb_interface_param_read_sub_ =
      nh_->subscribe("uwb_interface_param_read", 50,
                     &Ros1DataHandle::on_uwb_interface_param_read, this);
  ble_interface_param_read_sub_ =
      nh_->subscribe("ble_interface_param_read", 50,
                     &Ros1DataHandle::on_ble_interface_param_read, this);
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
  msg.anchors.resize(data.anchor_count);
  for (int i = 0; i < data.anchor_count; i++) {
    msg.anchors[i].addr = data.anchors[i].addr;
    msg.anchors[i].rx_rssi = data.anchors[i].rx_rssi;
    msg.anchors[i].rx_rate = data.anchors[i].rx_rate;
  }
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
  msg.is_sleeping = data.is_sleeping;
  msg.hardware_enabled_uart = data.hardware_enabled_uart;
  msg.hardware_enabled_iic = data.hardware_enabled_iic;
  msg.hardware_enabled_uwb = data.hardware_enabled_uwb;
  msg.hardware_enabled_ble = data.hardware_enabled_ble;
  msg.firmware_series = data.firmware_series;
  std::memcpy(msg.firmware_version.data(), data.firmware_version,
              sizeof(data.firmware_version));
  std::memcpy(msg.uid.data(), data.uid, UB_UID_SIZE);
  msg.battery_voltage = data.battery_voltage;
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

void Ros1DataHandle::dispatch(const UBDataAnchorDdoas &data) {
  ubeacon_driver::AnchorDdoas msg;
  msg.local_time_us = data.local_time_us;
  msg.datas.resize(data.count);
  for (int i = 0; i < data.count; i++) {
    msg.datas[i].a0 = data.datas[i].a0;
    msg.datas[i].a1 = data.datas[i].a1;
    msg.datas[i].ddoa = data.datas[i].ddoa;
  }
  anchor_ddoas_pub_.publish(msg);
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
  msg.sniff_duty_cycle = data.sniff_duty_cycle;
  msg.update_interval_max = data.update_interval_max;
  msg.reset_interval = data.reset_interval;
  param_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataRunTimeParam &data) {
  ubeacon_driver::RunTimeParam msg;
  msg.sniff_duty_cycle = data.sniff_duty_cycle;
  run_time_param_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataInterfaceParam &data) {
  ubeacon_driver::InterfaceParam msg;
  msg.uart = data.uart;
  msg.iic = data.iic;
  msg.uwb = data.uwb;
  msg.ble = data.ble;
  interface_param_pub_.publish(msg);
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
  msg.random_window = data.random_window;
  msg.max_rx_interval = data.max_rx_interval;
  uwb_interface_param_pub_.publish(msg);
}

void Ros1DataHandle::dispatch(const UBDataBleInterfaceParam &data) {
  ubeacon_driver::BleInterfaceParam msg;
  msg.company_id[0] = data.company_id[0];
  msg.company_id[1] = data.company_id[1];
  msg.data_type = data.data_type;
  msg.random_window = data.random_window;
  msg.send_count = data.send_count;
  ble_interface_param_pub_.publish(msg);
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
  data.sniff_duty_cycle = msg->sniff_duty_cycle;
  data.update_interval_max = msg->update_interval_max;
  data.reset_interval = msg->reset_interval;
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

void Ros1DataHandle::on_interface_param_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_INTERFACE_PARAM, nullptr, 0);
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

void Ros1DataHandle::on_ble_interface_param_read(
    const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  main_common_send_msg(UB_MSG_READ_BLE_INTERFACE_PARAM, nullptr, 0);
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
  case UB_MSG_ANCHOR_DDOAS:
    self->dispatch(*static_cast<const UBDataAnchorDdoas *>(data));
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
  case UB_MSG_INTERFACE_PARAM:
    self->dispatch(*static_cast<const UBDataInterfaceParam *>(data));
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
  case UB_MSG_BLE_INTERFACE_PARAM:
    self->dispatch(*static_cast<const UBDataBleInterfaceParam *>(data));
    break;
  default:
    break;
  }
}
