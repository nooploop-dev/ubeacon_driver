#pragma once

// 消息id及对应数据结构定义
// 此处的 UBData*
// 结构是"干净"的应用层结构：真实物理单位、float、bool、正常对齐。
// 线上真正传输的是打包定标后的字节（位域、定标整数、保留字段），已被驱动内部完全隐藏。
// 默认单位约定：时间 s，距离 m，速度 m/s（另有说明的以字段注释为准）

#ifdef __cplusplus
extern "C" {
#endif

#include "ubeacon_driver_common.h"

typedef enum {
  UB_MSG_RESTART = 2,
  UB_MSG_FIND = 3,

  UB_MSG_ANCHOR_POS = 51,
  UB_MSG_GLOBAL_TIME_STATUS = 58,

  UB_MSG_READ_PARAM = 60,
  UB_MSG_WRITE_PARAM = 61,
  UB_MSG_PARAM = UB_MSG_WRITE_PARAM,

  UB_MSG_READ_INTERFACE_PARAM = 62,
  UB_MSG_WRITE_INTERFACE_PARAM = 63,
  UB_MSG_INTERFACE_PARAM = UB_MSG_WRITE_INTERFACE_PARAM,

  UB_MSG_READ_UART_INTERFACE_PARAM = 64,
  UB_MSG_WRITE_UART_INTERFACE_PARAM = 65,
  UB_MSG_UART_INTERFACE_PARAM = UB_MSG_WRITE_UART_INTERFACE_PARAM,

  UB_MSG_READ_IIC_INTERFACE_PARAM = 66,
  UB_MSG_WRITE_IIC_INTERFACE_PARAM = 67,
  UB_MSG_IIC_INTERFACE_PARAM = UB_MSG_WRITE_IIC_INTERFACE_PARAM,

  UB_MSG_LOCATION_RESULT = 68,

  UB_MSG_HEARTBEAT = 78,

  UB_MSG_READ_UWB_INTERFACE_PARAM = 79,
  UB_MSG_WRITE_UWB_INTERFACE_PARAM = 80,
  UB_MSG_UWB_INTERFACE_PARAM = UB_MSG_WRITE_UWB_INTERFACE_PARAM,

  UB_MSG_USER_DATA = 81,

  UB_MSG_ANCHOR_SIGNAL = 96,

  UB_MSG_READ_RUN_TIME_PARAM = 100,
  UB_MSG_WRITE_RUN_TIME_PARAM = 101,
  UB_MSG_RUN_TIME_PARAM = UB_MSG_WRITE_RUN_TIME_PARAM,

  UB_MSG_READ_BLE_INTERFACE_PARAM = 102,
  UB_MSG_WRITE_BLE_INTERFACE_PARAM = 103,
  UB_MSG_BLE_INTERFACE_PARAM = UB_MSG_WRITE_BLE_INTERFACE_PARAM,

  UB_MSG_ANCHOR_DDOAS = 104,

  UB_MSG_Z_MEASUREMENT = 111,

  UB_MSG_STATE_CONTROL = 112,
  UB_MSG_MAP_MEASUREMENT = 113,
} ub_msg_e;

#define UB_LOCATION_RESULT_ANCHOR_MAX 9
#define UB_ANCHOR_SIGNAL_DATA_MAX 9
#define UB_ANCHOR_DDOAS_DATA_MAX 10
#define UB_USER_DATA_PAYLOAD_MAX 64

// 重启设备，部分配置参数变更后需要重启才会生效
typedef struct {
  // 延迟时间 s
  uint8_t delay;
  // 仅在设备本身需要时重启
  bool only_restart_when_need;
} UBDataRestart;

// 寻找设备
typedef struct {
  // 设备收到后持续对外提示（震动、闪灯）的时间 s
  uint8_t duration;
} UBDataFind;

// ROM 工作参数
typedef struct {
  // 标签期望高度，即大部分状态下距地面的高度
  float expect_z;
  // 期望高度的标准差，高度变化大则调大，但不确定性会影响 xy
  float z_noise;
  // 滤波数据平滑窗口，越大越平滑但延迟越大，0~5
  uint8_t smooth_window;
  // 大部分状态下的最大加速度
  float max_acceleration[3];
  struct {
    // 输出标签定位结果
    bool tag_pos;
    // 转发信标发送给后台的数据包
    bool anchor_packet;
    // 输出标签收到的信标坐标信息
    bool anchor_pos;
    // 输出信标之间链路信息
    bool anchor_link_data;
    // 输出标签收到的信标定位信号数据
    bool anchor_signal;
    // 输出标签到信标间的距离差信息
    bool anchor_ddoa;
    // 即使定位结果无效也输出
    bool tag_pos_even_error;
    // 输出信标间链路同步状态
    bool anchor_link_status;
  } output;
  // 未收到信标信号时周期嗅探的默认占空比，1~100
  uint8_t sniff_duty_cycle;
  // 前后两次更新的最大间隔时间 s
  uint8_t update_interval_max;
  // 前后两次更新超过此间隔将触发滤波重置 s
  uint16_t reset_interval;
} UBDataParam;

// RAM 运行时工作参数
typedef struct {
  // 复位后初始化为 Param.sniff_duty_cycle，之后可随时修改
  uint8_t sniff_duty_cycle;
} UBDataRunTimeParam;

// 定位结果
typedef struct {
  ub_local_time_us_t local_time_us;
  // 位置
  float pos[3];
  // 速度
  float vel[3];
  // 位置噪声标准差
  float pos_noise[3];
  // 速度噪声标准差
  float vel_noise[3];
  uint8_t map_id;
  // 使能 tag_pos_even_error 时定位失败仍输出，此时非零
  uint8_t error_code;
  uint8_t area_id;
  // anchors 中的有效元素数量
  uint8_t anchor_count;
  struct {
    ub_addr_t addr;
    // 接收信号强度 dBm
    float rx_rssi;
    // 收包率 0~1
    float rx_rate;
  } anchors[UB_LOCATION_RESULT_ANCHOR_MAX];
} UBDataLocationResult;

// ROM 标签对外通信方式软件配置
// 最终启用情况由硬件 IO 配置与软件配置共同决定（或关系），保底启用 uwb
typedef struct {
  bool uart;
  bool iic;
  bool uwb;
  bool ble;
} UBDataInterfaceParam;

// ROM 串口通信参数
typedef struct {
  uint32_t baudrate;
} UBDataUartInterfaceParam;

// ROM iic 通信参数
typedef struct {
  // 从机地址
  uint8_t addr;
} UBDataIicInterfaceParam;

// ROM uwb 通信参数
typedef struct {
  // 发送信号的随机窗口，避免同时发送干扰
  float random_window;
  // 两次打开接收的最大间隔，0 表示不限制
  float max_rx_interval;
} UBDataUwbInterfaceParam;

// ROM ble 通信参数
typedef struct {
  // 广播数据帧中的 company_id
  uint8_t company_id[2];
  uint8_t data_type;
  // 发送信号的随机窗口
  float random_window;
  // 每帧数据连续发送次数
  uint8_t send_count;
} UBDataBleInterfaceParam;

// 心跳消息，5s 定时发送到后台
typedef struct {
  // 电量百分比
  uint8_t battery_percent;
  bool battery_charging;
  bool need_restart;
  // 是否存在新的重启信息
  bool reset_info_dirty;
  // 是否存在新的断言信息
  bool assert_info_dirty;
  // 任何原因重启计数都会 +1
  uint8_t restart_cnt;
  // 收到 StateControl 后跟随其 sleep 变化
  bool is_sleeping;
  // 用户通过硬件 IO 选定使能的接口情况
  bool hardware_enabled_uart;
  bool hardware_enabled_iic;
  bool hardware_enabled_uwb;
  bool hardware_enabled_ble;
  uint8_t firmware_series;
  uint8_t firmware_version[4];
  uint8_t uid[UB_UID_SIZE];
  // 如使能电池，返回电压 V
  float battery_voltage;
} UBDataHeartbeat;

// 用户数传
typedef struct {
  uint8_t payload_size;
  uint8_t payload[UB_USER_DATA_PAYLOAD_MAX];
} UBDataUserData;

// 每次定位对应的所有信标信号数据
typedef struct {
  ub_local_time_us_t local_time_us;
  // datas 中的有效元素数量
  uint8_t count;
  // 对应的区域 id
  uint8_t area_id;
  struct {
    ub_addr_t addr;
    float fp_index;
    int8_t fp_to_peak;
    float mc;
    // 接收信号强度 dBm
    float rx_rssi;
    // 第一路径信号强度 dBm
    float fp_rssi;
    float uwb_clock_offset_ppm;
    float mcu_clock_offset_ppm;
    // 近期该信标信号的收包率 0~1
    float rx_rate;
  } datas[UB_ANCHOR_SIGNAL_DATA_MAX];
} UBDataAnchorSignal;

// 信标之间的距离差数据
typedef struct {
  ub_local_time_us_t local_time_us;
  // datas 中的有效元素数量
  uint8_t count;
  struct {
    ub_addr_t a0;
    ub_addr_t a1;
    // ddoa = dis_tag_to_a1 - dis_tag_to_a0，单位 m
    float ddoa;
  } datas[UB_ANCHOR_DDOAS_DATA_MAX];
} UBDataAnchorDdoas;

// 用户有准确的标签 z 坐标时可通过此消息输入，可提升定位精度、降低地图切换延迟
typedef struct {
  // 距离零平面的相对高度 m
  float z;
  // 高度信息噪声标准差 m
  float z_std;
  // 本次测量值超时时间 s，超时后退回默认固定高度状态
  uint8_t timeout;
} UBDataZMeasurement;

// 控制标签工作状态
typedef struct {
  // false 唤醒，true 睡眠
  bool sleep;
} UBDataStateControl;

// 用户如有其他方式知道自己在哪个地图，可输入以优化定位
typedef struct {
  uint8_t map_id;
  // 测量值超时时间 s
  uint8_t timeout;
} UBDataMapMeasurement;

// 后台启用全局时间同步后，标签跟随心跳发送此消息
typedef struct {
  // 是否处于时间同步状态
  bool run;
  // 超过此时间未收到此消息可认为已退出同步状态，0 表示不自动退出 s
  uint8_t timeout;
  // 标签获取全局时间的中间跳数，跳数越大误差越大
  uint8_t ttl;
  // 全局时间 us
  uint64_t time_us;
  // 当前节点时间同步跟随的目标设备
  ub_addr_t src;
} UBDataGlobalTimeStatus;

// 标签收到的信标坐标信息
typedef struct {
  ub_local_time_us_t local_time_us;
  ub_addr_t src;
  // m
  float pos[3];
  bool is_local_pos;
  uint8_t map_id;
  // map_z = pos[2] + relative_map_z，单位 m
  float relative_map_z;
} UBDataAnchorPos;

#ifdef __cplusplus
}
#endif
