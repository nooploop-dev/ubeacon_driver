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

// user表示用户侧，dev表示设备侧
// 消息方向：v:user>dev, ^:dev>user，
// 有消息ID但没有对应结构体定义的为空消息（payload_size为0）
// write消息和响应消息通常共用一个ID，结构体也共用
// 单个方向上，每个消息ID都是唯一的，对应唯一的 UBData* 结构；
enum {
  UB_MSG_RESTART = 2, // v^
  UB_MSG_FIND = 3,    // v^

  UB_MSG_ANCHOR_POS = 51,         // ^
  UB_MSG_GLOBAL_TIME_STATUS = 58, // ^

  UB_MSG_READ_PARAM = 60,            // v
  UB_MSG_WRITE_PARAM = 61,           // v
  UB_MSG_PARAM = UB_MSG_WRITE_PARAM, // ^

  UB_MSG_READ_PERIPHERAL_CONFIG = 62,                        // v
  UB_MSG_WRITE_PERIPHERAL_CONFIG = 63,                       // v
  UB_MSG_PERIPHERAL_CONFIG = UB_MSG_WRITE_PERIPHERAL_CONFIG, // ^

  UB_MSG_READ_UART_INTERFACE_PARAM = 64,                           // v
  UB_MSG_WRITE_UART_INTERFACE_PARAM = 65,                          // v
  UB_MSG_UART_INTERFACE_PARAM = UB_MSG_WRITE_UART_INTERFACE_PARAM, // ^

  UB_MSG_READ_IIC_INTERFACE_PARAM = 66,                          // v
  UB_MSG_WRITE_IIC_INTERFACE_PARAM = 67,                         // v
  UB_MSG_IIC_INTERFACE_PARAM = UB_MSG_WRITE_IIC_INTERFACE_PARAM, // ^

  UB_MSG_LOCATION_RESULT = 68, // ^

  UB_MSG_HEARTBEAT = 78, // ^

  UB_MSG_READ_UWB_INTERFACE_PARAM = 79,                          // v
  UB_MSG_WRITE_UWB_INTERFACE_PARAM = 80,                         // v
  UB_MSG_UWB_INTERFACE_PARAM = UB_MSG_WRITE_UWB_INTERFACE_PARAM, // ^

  UB_MSG_USER_DATA = 81, // v^

  UB_MSG_ANCHOR2TAG_SIGNAL = 89, // ^
  UB_MSG_TAG2ANCHOR_DIS = 91,    // ^
  UB_MSG_TAG2ANCHOR_PDOA = 92,   // ^

  UB_MSG_ANCHOR_SIGNAL = 96, // ^

  UB_MSG_READ_RUN_TIME_PARAM = 100,                    // v
  UB_MSG_WRITE_RUN_TIME_PARAM = 101,                   // v
  UB_MSG_RUN_TIME_PARAM = UB_MSG_WRITE_RUN_TIME_PARAM, // ^

  UB_MSG_ANCHOR_DDOAS = 104, // ^

  UB_MSG_READ_HARDWARE_PERIPHERAL_CONFIG = 105, // v
  UB_MSG_HARDWARE_PERIPHERAL_CONFIG = 106,      // ^
  UB_MSG_READ_FINAL_PERIPHERAL_CONFIG = 107,    // v
  UB_MSG_FINAL_PERIPHERAL_CONFIG = 108,         // ^

  UB_MSG_Z_MEASUREMENT = 111, // v^

  UB_MSG_STATE_CONTROL = 112,   // v^
  UB_MSG_MAP_MEASUREMENT = 113, // v^

  UB_MSG_IMU = 115,      // ^
  UB_MSG_ATTITUDE = 116, // ^
};

#define UB_LOCATION_RESULT_ANCHOR_MAX 9
#define UB_ANCHOR_SIGNAL_DATA_MAX 9
#define UB_ANCHOR2TAG_SIGNAL_DATA_MAX 9
#define UB_TAG2ANCHOR_DIS_DATA_MAX 9
#define UB_TAG2ANCHOR_PDOA_DATA_MAX 9
#define UB_TAG2ANCHOR_PDOA_COUNT_MAX 4
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
    // 输出 imu 原始数据
    bool imu;
    // 输出姿态数据
    bool attitude;
    // 输出标签到信标的测距结果
    bool dis;
    // 输出标签到信标的测角结果
    bool pdoa;
  } output;
  // 未收到信标信号时周期嗅探的默认占空比，百分比 1~100
  uint8_t sniff_duty_cycle;
  // 前后两次更新的最大间隔时间 s
  uint8_t update_interval_max;
  // 前后两次更新超过此间隔将触发滤波重置 s
  uint16_t reset_interval;
  // 收到信号的信标数量少于此值时启用嗅探
  uint8_t min_sync_anchor_count;
  // 嗅探占空比下限，百分比
  // 0~100；非零表示即使已与足够多信标同步也周期嗅探，以便发现新信标
  uint8_t min_sniff_duty_cycle;
  // 定位结果中不再附带参与定位的信标列表（LocationResult.anchor_count 为 0）
  bool disable_anchor_data_output_with_result;
  // 标签固定在某个地图活动时可指定，避免受其他地图影响，0 表示不固定
  uint8_t static_map_id;
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

// ROM 外设软件配置
// 同一结构用于三个消息：PERIPHERAL_CONFIG 为软件配置（可读写），
// HARDWARE_PERIPHERAL_CONFIG 为硬件电路指定的配置（只读），
// FINAL_PERIPHERAL_CONFIG 为两者合并后的最终启用情况（只读）
typedef struct {
  // 通信接口
  bool uart;
  bool iic;
  bool uwb;
  bool ble;
  bool four_g;
  // 软件配置的 priority 高于硬件配置时，软件配置完全覆盖硬件配置；
  // 否则最终配置由软硬件配置共同决定（或关系），保底启用 uwb
  bool priority;
  // 外设
  bool motor;
  bool bee;
  bool key;
  bool motion;
  bool battery;
  bool battery_chargeable;
  bool baro;
  bool imu;
  bool gnss;
} UBDataPeripheralConfig;

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
  // 发送信号的随机窗口 ms，避免同时发送干扰，0 表示不延迟立即发送，1~5000
  uint16_t random_window_ms;
  // 两次打开接收的最大间隔 ms，0 表示不限制，每次发送信号后都打开接收；50ms 分辨率，上限 12750
  uint16_t max_rx_interval_ms;
  // 标签数量较多、频率较高时，限制标签 uwb 回传的最小间隔 ms
  uint16_t min_tx_interval_ms;
  // 标签主动发起测距的频率 Hz
  float dis_frequency;
  // 标签主动发起测距允许的最大时隙数量，越大支持同时测距的信标越多但功耗越高，
  // >0 表示启用测距，上限 18
  uint8_t dis_slot_count_max;
  // 末尾保留不分配的时隙数量，用于新信标随机接入，>=1，dis_slot_count_max>0
  // 时有效
  uint8_t dis_remain_slot_count;
} UBDataUwbInterfaceParam;

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
  bool hardware_enabled_four_g;
  uint8_t firmware_series;
  uint8_t firmware_version[4];
  uint8_t uid[UB_UID_SIZE];
  // 如使能电池，返回电压 V
  float battery_voltage;
  // 近期 uwb 运行事件标志，用于诊断通信环境
  // 收到 sts 校验错误的信号，通常来自非本系统（sts key 不匹配）
  bool recent_rx_sts_error_occurred;
  // 收到数据校验/解密错误的信号，通常来自非本系统
  bool recent_rx_data_error_occurred;
  // 全局时间同步后发生倒退，通常意味着全局时间同步不稳定
  bool recent_global_time_error_occurred;
  // 收到的其他节点全局时间不可靠
  bool recent_peer_global_time_error_occurred;
  // 收到的其他节点本地时间不可靠
  bool recent_peer_local_time_error_occurred;
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

// 以下几个消息对应信标信号成组先后到达的一次定位/测距：
// begin_time_us 为组内首个信号的时间，time_slot_duration_us
// 为组内相邻信号的间隔， 第 i 个元素对应的信号时间 = begin_time_us + i *
// time_slot_duration_us

// 信标之间的距离差数据
typedef struct {
  ub_local_time_us_t begin_time_us;
  uint16_t time_slot_duration_us;
  // datas 中的有效元素数量
  uint8_t count;
  struct {
    ub_addr_t a0;
    ub_addr_t a1;
    // ddoa = dis_tag_to_a1 - dis_tag_to_a0，单位 m
    float ddoa;
  } datas[UB_ANCHOR_DDOAS_DATA_MAX];
} UBDataAnchorDdoas;

// 每次收到需要参与定位的信标信号后输出的信号数据
typedef struct {
  ub_local_time_us_t begin_time_us;
  uint16_t time_slot_duration_us;
  // datas 中的有效元素数量
  uint8_t count;
  struct {
    ub_addr_t addr;
    // 接收信号强度 dBm
    float rx_rssi;
    // 第一路径信号强度 dBm
    float fp_rssi;
    float uwb_clock_offset_ppm;
    float mcu_clock_offset_ppm;
    // 近期该信标信号的收包率 0~1
    float rx_rate;
  } datas[UB_ANCHOR2TAG_SIGNAL_DATA_MAX];
} UBDataAnchor2TagSignal;

// 标签与信标之间的测距结果
typedef struct {
  ub_local_time_us_t begin_time_us;
  uint16_t time_slot_duration_us;
  // datas 中的有效元素数量
  uint8_t count;
  struct {
    ub_addr_t addr;
    // m
    float dis;
  } datas[UB_TAG2ANCHOR_DIS_DATA_MAX];
} UBDataTag2AnchorDis;

// 标签与信标之间的测角结果
typedef struct {
  ub_local_time_us_t begin_time_us;
  uint16_t time_slot_duration_us;
  // datas 中的有效元素数量
  uint8_t count;
  struct {
    ub_addr_t addr;
    // 信标天线类型，0 为非阵列天线，不支持测角
    uint8_t antenna_type;
    // pdoas 中的有效元素数量
    uint8_t pdoa_count;
    // 相位差，量程 ±3.2767
    float pdoas[UB_TAG2ANCHOR_PDOA_COUNT_MAX];
  } datas[UB_TAG2ANCHOR_PDOA_DATA_MAX];
} UBDataTag2AnchorPdoa;

// imu 原始数据
typedef struct {
  ub_local_time_us_t local_time_us;
  // 角速度 rad/s
  float gyro[3];
  // 加速度 m/s^2
  float acc[3];
} UBDataImu;

// 姿态数据
typedef struct {
  ub_local_time_us_t local_time_us;
  // 四元数 (w, x, y, z)
  float q[4];
  // 姿态噪声标准差 (roll, pitch, yaw) rad
  float noise[3];
} UBDataAttitude;

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
