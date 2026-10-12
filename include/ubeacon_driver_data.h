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
  UB_MSG_RESTART = 2,     // v^
  UB_MSG_FIND = 3,        // v^
  UB_MSG_RESET_PARAM = 4, // v^

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

  UB_MSG_READ_TWR_PARAM = 69,                // v
  UB_MSG_WRITE_TWR_PARAM = 70,               // v
  UB_MSG_TWR_PARAM = UB_MSG_WRITE_TWR_PARAM, // ^

  UB_MSG_HEARTBEAT = 78, // ^

  UB_MSG_READ_UWB_INTERFACE_PARAM = 79,                          // v
  UB_MSG_WRITE_UWB_INTERFACE_PARAM = 80,                         // v
  UB_MSG_UWB_INTERFACE_PARAM = UB_MSG_WRITE_UWB_INTERFACE_PARAM, // ^

  UB_MSG_USER_DATA = 81, // v^

  UB_MSG_ANCHOR2TAG_SIGNAL = 90, // ^
  UB_MSG_TAG2ANCHOR_DIS = 91,    // ^
  UB_MSG_TAG2ANCHOR_PDOA = 92,   // ^

  UB_MSG_READ_RUN_TIME_PARAM = 100,                    // v
  UB_MSG_WRITE_RUN_TIME_PARAM = 101,                   // v
  UB_MSG_RUN_TIME_PARAM = UB_MSG_WRITE_RUN_TIME_PARAM, // ^

  UB_MSG_ANCHOR2TAG_DDOA = 104, // ^

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

#define UB_ANCHOR2TAG_SIGNAL_DATA_MAX 9
#define UB_TAG2ANCHOR_DIS_DATA_MAX 9
#define UB_TAG2ANCHOR_PDOA_DATA_MAX 9
#define UB_TAG2ANCHOR_PDOA_COUNT_MAX 4
#define UB_ANCHOR2TAG_DDOA_DATA_MAX 10
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
  // 两次打开接收的最大间隔 ms，0 表示不限制，每次发送信号后都打开接收；50ms
  // 分辨率，上限 12750
  uint16_t max_rx_interval_ms;
  // 标签数量较多、频率较高时，限制标签 uwb 回传的最小间隔 ms
  uint16_t min_tx_interval_ms;
} UBDataUwbInterfaceParam;

// ROM 标签主动测距参数，测距与 uwb 回传是相互独立的功能
typedef struct {
  // 主动发起测距的频率 Hz，与 dis_slot_count_max 同时 >0 时启用测距
  float dis_frequency;
  // 每轮测距允许的最大时隙数量，越大支持同时测距的信标越多但功耗越高，上限 9
  uint8_t dis_slot_count_max;
  // 末尾保留不分配的时隙数量，用于新信标随机接入，>=1
  uint8_t dis_remain_slot_count;
} UBDataTwrParam;

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
  // 用户是否通过 StateControl 请求睡眠，跟随其 sleep 变化；
  // 设备实际是否工作还受电源开关、静止检测、固件更新等影响
  bool user_request_sleep;
  // 外设最终使能情况，即软件配置与硬件配置合并后的结果，同
  // FINAL_PERIPHERAL_CONFIG
  bool hardware_enabled_motion;
  bool hardware_enabled_uart;
  bool hardware_enabled_iic;
  bool hardware_enabled_uwb;
  bool hardware_enabled_ble;
  bool hardware_enabled_four_g;
  bool hardware_enabled_baro;
  bool hardware_enabled_imu;
  bool hardware_enabled_gnss;
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
  // 设备自检标志：rtos / mcu / uwb 模块可能异常
  bool rtos_warn;
  bool mcu_warn;
  bool uwb_warn;
} UBDataHeartbeat;

// 用户数传
typedef struct {
  uint8_t payload_size;
  uint8_t payload[UB_USER_DATA_PAYLOAD_MAX];
} UBDataUserData;

// 标签到信标间的距离差数据。一次定位的全部距离差可能拆成多条消息输出，
// 同一次定位的各条消息 begin_time_us 相同，由 group_index / group_count 区分
typedef struct {
  // 对应定位结果的 local_time_us
  ub_local_time_us_t begin_time_us;
  // 本次定位拆分出的消息条数
  uint8_t group_count;
  // 本条消息的序号，从 0 开始
  uint8_t group_index;
  // datas 中的有效元素数量
  uint8_t count;
  struct {
    ub_addr_t a0;
    ub_addr_t a1;
    // ddoa = dis_tag_to_a1 - dis_tag_to_a0，单位 m
    float ddoa;
  } datas[UB_ANCHOR2TAG_DDOA_DATA_MAX];
} UBDataAnchor2TagDdoa;

// Anchor2TagSignal 中信号来源的定位模式，决定 begin_time_us 与 rx_rate 的含义
typedef enum {
  // begin_time_us 为本轮 ccp 的开始时间，rx_rate 为 ccp 收包率
  UB_LOCATION_MODE_TDOA,
  // begin_time_us 为标签发出 poll 的时刻，各信标应答在其后的时隙内到达，
  // rx_rate 为测距应答收包率
  UB_LOCATION_MODE_TWR,
  // begin_time_us 为该信号的接收时刻，rx_rate 为信标广播的收包率，count 恒为 1
  UB_LOCATION_MODE_RSSI,
} ub_location_mode_e;

// 每次收到需要参与定位的信标信号后输出的信号数据
typedef struct {
  ub_local_time_us_t begin_time_us;
  // datas 中的有效元素数量
  uint8_t count;
  // ub_location_mode_e
  uint8_t location_mode;
  struct {
    ub_addr_t addr;
    // 接收信号强度 dBm
    float rx_rssi;
    // 第一路径信号强度 dBm
    float fp_rssi;
    // 频偏取整到 1ppm，超出 ±127 时饱和到 ±127，NaN 表示无效
    float uwb_clock_offset_ppm;
    float mcu_clock_offset_ppm;
    // 近期该信标信号的收包率 0~1
    float rx_rate;
  } datas[UB_ANCHOR2TAG_SIGNAL_DATA_MAX];
} UBDataAnchor2TagSignal;

// 标签与信标之间的测距结果
typedef struct {
  // 本轮测距的发起时间
  ub_local_time_us_t begin_time_us;
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
  // 本轮测距的发起时间
  ub_local_time_us_t begin_time_us;
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
