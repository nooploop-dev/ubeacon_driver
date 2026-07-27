#pragma once

#include "ubeacon_driver_for_user.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ubeacon_driver/AnchorDdoas.h>
#include <ubeacon_driver/AnchorPos.h>
#include <ubeacon_driver/AnchorSignal.h>
#include <ubeacon_driver/BleInterfaceParam.h>
#include <ubeacon_driver/Find.h>
#include <ubeacon_driver/GlobalTimeStatus.h>
#include <ubeacon_driver/Heartbeat.h>
#include <ubeacon_driver/IicInterfaceParam.h>
#include <ubeacon_driver/InterfaceParam.h>
#include <ubeacon_driver/LocationResult.h>
#include <ubeacon_driver/MapMeasurement.h>
#include <ubeacon_driver/Param.h>
#include <ubeacon_driver/Restart.h>
#include <ubeacon_driver/RunTimeParam.h>
#include <ubeacon_driver/StateControl.h>
#include <ubeacon_driver/UartInterfaceParam.h>
#include <ubeacon_driver/UserData.h>
#include <ubeacon_driver/UwbInterfaceParam.h>
#include <ubeacon_driver/ZMeasurement.h>

class Ros1DataHandle {
public:
  explicit Ros1DataHandle(ros::NodeHandle *nh);

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
  void on_find(const ubeacon_driver::Find::ConstPtr &msg);
  void on_restart(const ubeacon_driver::Restart::ConstPtr &msg);
  void on_state_control(const ubeacon_driver::StateControl::ConstPtr &msg);
  void on_z_measurement(const ubeacon_driver::ZMeasurement::ConstPtr &msg);
  void on_map_measurement(const ubeacon_driver::MapMeasurement::ConstPtr &msg);
  void on_user_data_to_device(const ubeacon_driver::UserData::ConstPtr &msg);
  void on_param_read(const std_msgs::Empty::ConstPtr &msg);
  void on_param_write(const ubeacon_driver::Param::ConstPtr &msg);
  void on_run_time_param_read(const std_msgs::Empty::ConstPtr &msg);
  void on_run_time_param_write(const ubeacon_driver::RunTimeParam::ConstPtr &msg);
  void on_interface_param_read(const std_msgs::Empty::ConstPtr &msg);
  void on_uart_interface_param_read(const std_msgs::Empty::ConstPtr &msg);
  void on_iic_interface_param_read(const std_msgs::Empty::ConstPtr &msg);
  void on_uwb_interface_param_read(const std_msgs::Empty::ConstPtr &msg);
  void on_ble_interface_param_read(const std_msgs::Empty::ConstPtr &msg);

  UBParserFromDev parser_;
  ros::NodeHandle *nh_;
  std::string frame_id_;

  ros::Publisher location_result_pub_;
  ros::Publisher location_result_pose_pub_;
  ros::Publisher heartbeat_pub_;
  ros::Publisher anchor_signal_pub_;
  ros::Publisher anchor_ddoas_pub_;
  ros::Publisher anchor_pos_pub_;
  ros::Publisher global_time_status_pub_;
  ros::Publisher user_data_from_device_pub_;
  ros::Publisher param_pub_;
  ros::Publisher run_time_param_pub_;
  ros::Publisher interface_param_pub_;
  ros::Publisher uart_interface_param_pub_;
  ros::Publisher iic_interface_param_pub_;
  ros::Publisher uwb_interface_param_pub_;
  ros::Publisher ble_interface_param_pub_;

  ros::Subscriber find_sub_;
  ros::Subscriber restart_sub_;
  ros::Subscriber state_control_sub_;
  ros::Subscriber z_measurement_sub_;
  ros::Subscriber map_measurement_sub_;
  ros::Subscriber user_data_to_device_sub_;
  ros::Subscriber param_read_sub_;
  ros::Subscriber param_write_sub_;
  ros::Subscriber run_time_param_read_sub_;
  ros::Subscriber run_time_param_write_sub_;
  ros::Subscriber interface_param_read_sub_;
  ros::Subscriber uart_interface_param_read_sub_;
  ros::Subscriber iic_interface_param_read_sub_;
  ros::Subscriber uwb_interface_param_read_sub_;
  ros::Subscriber ble_interface_param_read_sub_;
};
