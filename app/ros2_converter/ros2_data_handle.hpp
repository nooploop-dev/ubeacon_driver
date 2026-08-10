#pragma once

#include "ubeacon_driver_for_user.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <ubeacon_driver/msg/anchor_ddoas.hpp>
#include <ubeacon_driver/msg/anchor_pos.hpp>
#include <ubeacon_driver/msg/anchor_signal.hpp>
#include <ubeacon_driver/msg/ble_interface_param.hpp>
#include <ubeacon_driver/msg/find.hpp>
#include <ubeacon_driver/msg/global_time_status.hpp>
#include <ubeacon_driver/msg/heartbeat.hpp>
#include <ubeacon_driver/msg/iic_interface_param.hpp>
#include <ubeacon_driver/msg/interface_param.hpp>
#include <ubeacon_driver/msg/location_result.hpp>
#include <ubeacon_driver/msg/map_measurement.hpp>
#include <ubeacon_driver/msg/param.hpp>
#include <ubeacon_driver/msg/restart.hpp>
#include <ubeacon_driver/msg/run_time_param.hpp>
#include <ubeacon_driver/msg/state_control.hpp>
#include <ubeacon_driver/msg/uart_interface_param.hpp>
#include <ubeacon_driver/msg/user_data.hpp>
#include <ubeacon_driver/msg/uwb_interface_param.hpp>
#include <ubeacon_driver/msg/z_measurement.hpp>

class Ros2DataHandle {
public:
  explicit Ros2DataHandle(const rclcpp::Node::SharedPtr &node);

  UBParserFromDev *parser() { return &parser_; }

private:
  // 收到设备消息的回调(C接口，arg即this)
  static void on_frame_msg(void *arg, ub_msg_id_t msg_id, const void *data,
                           int data_size);
  // 设备 -> 用户(^)，解析后发布到话题
  void dispatch(const UBDataLocationResult &data);
  void dispatch(const UBDataHeartbeat &data);
  void dispatch(const UBDataAnchorSignal &data);
  void dispatch(const UBDataAnchorDdoas &data);
  void dispatch(const UBDataAnchorPos &data);
  void dispatch(const UBDataGlobalTimeStatus &data);
  void dispatch(const UBDataUserData &data);
  void dispatch(const UBDataParam &data);
  void dispatch(const UBDataRunTimeParam &data);
  void dispatch(const UBDataInterfaceParam &data);
  void dispatch(const UBDataUartInterfaceParam &data);
  void dispatch(const UBDataIicInterfaceParam &data);
  void dispatch(const UBDataUwbInterfaceParam &data);
  void dispatch(const UBDataBleInterfaceParam &data);

  // 用户 -> 设备(v)，订阅话题后发送给设备
  void on_find(const ubeacon_driver::msg::Find::SharedPtr msg);
  void on_restart(const ubeacon_driver::msg::Restart::SharedPtr msg);
  void
  on_state_control(const ubeacon_driver::msg::StateControl::SharedPtr msg);
  void
  on_z_measurement(const ubeacon_driver::msg::ZMeasurement::SharedPtr msg);
  void
  on_map_measurement(const ubeacon_driver::msg::MapMeasurement::SharedPtr msg);
  void
  on_user_data_to_device(const ubeacon_driver::msg::UserData::SharedPtr msg);
  void on_param_read(const std_msgs::msg::Empty::SharedPtr msg);
  void on_param_write(const ubeacon_driver::msg::Param::SharedPtr msg);
  void on_run_time_param_read(const std_msgs::msg::Empty::SharedPtr msg);
  void on_run_time_param_write(
      const ubeacon_driver::msg::RunTimeParam::SharedPtr msg);
  void on_interface_param_read(const std_msgs::msg::Empty::SharedPtr msg);
  void on_uart_interface_param_read(const std_msgs::msg::Empty::SharedPtr msg);
  void on_iic_interface_param_read(const std_msgs::msg::Empty::SharedPtr msg);
  void on_uwb_interface_param_read(const std_msgs::msg::Empty::SharedPtr msg);
  void on_ble_interface_param_read(const std_msgs::msg::Empty::SharedPtr msg);

  UBParserFromDev parser_;
  rclcpp::Node::SharedPtr node_;
  std::string frame_id_;

  rclcpp::Publisher<ubeacon_driver::msg::LocationResult>::SharedPtr
      location_result_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      location_result_pose_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::Heartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::AnchorSignal>::SharedPtr
      anchor_signal_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::AnchorDdoas>::SharedPtr
      anchor_ddoas_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::AnchorPos>::SharedPtr anchor_pos_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::GlobalTimeStatus>::SharedPtr
      global_time_status_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::UserData>::SharedPtr
      user_data_from_device_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::Param>::SharedPtr param_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::RunTimeParam>::SharedPtr
      run_time_param_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::InterfaceParam>::SharedPtr
      interface_param_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::UartInterfaceParam>::SharedPtr
      uart_interface_param_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::IicInterfaceParam>::SharedPtr
      iic_interface_param_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::UwbInterfaceParam>::SharedPtr
      uwb_interface_param_pub_;
  rclcpp::Publisher<ubeacon_driver::msg::BleInterfaceParam>::SharedPtr
      ble_interface_param_pub_;

  rclcpp::Subscription<ubeacon_driver::msg::Find>::SharedPtr find_sub_;
  rclcpp::Subscription<ubeacon_driver::msg::Restart>::SharedPtr restart_sub_;
  rclcpp::Subscription<ubeacon_driver::msg::StateControl>::SharedPtr
      state_control_sub_;
  rclcpp::Subscription<ubeacon_driver::msg::ZMeasurement>::SharedPtr
      z_measurement_sub_;
  rclcpp::Subscription<ubeacon_driver::msg::MapMeasurement>::SharedPtr
      map_measurement_sub_;
  rclcpp::Subscription<ubeacon_driver::msg::UserData>::SharedPtr
      user_data_to_device_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr param_read_sub_;
  rclcpp::Subscription<ubeacon_driver::msg::Param>::SharedPtr param_write_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      run_time_param_read_sub_;
  rclcpp::Subscription<ubeacon_driver::msg::RunTimeParam>::SharedPtr
      run_time_param_write_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      interface_param_read_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      uart_interface_param_read_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      iic_interface_param_read_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      uwb_interface_param_read_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      ble_interface_param_read_sub_;
};
