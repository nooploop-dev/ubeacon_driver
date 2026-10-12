#pragma once

// 内部头文件：线格式（wire format）结构与转换器，用户不应包含此文件
//
// UBRawData* 是真正在链路上传输的字节布局：pack(1)、位域、定标整数、保留字段。
// 每个消息提供一对纯函数在 UBData*（应用层）与 UBRawData*（线格式）之间转换：
//   ub_data_xxx_from_raw()  上行：解析收到的字节
//   ub_data_xxx_to_raw()    下行：构造要发送的字节，并输出实际长度（变长消息按有效元素数截断）
//
// 所有定标魔数集中在本文件，应用层只读写真实物理单位。

#ifdef __cplusplus
extern "C" {
#endif

#include "ubeacon_driver_data.h"
#include <math.h> // 仅用 NAN 宏，不调用任何 libm 函数
#include <stddef.h>
#include <string.h>

// ---------------- 转换辅助 ----------------

// 四舍五入，自己实现以免链接 libm
static inline int32_t ub_round_i32(float v) {
  return (int32_t)(v < 0.0f ? v - 0.5f : v + 0.5f);
}

static inline int ub_min_int(int a, int b) { return a < b ? a : b; }

// 整数 ppm 频偏的线上无效值，有效值只用 [-127, 127]
#define UB_PPM_INT8_INVALID (-128)

static inline float ub_ppm_from_int8(int8_t v) {
  return v == UB_PPM_INT8_INVALID ? NAN : (float)v;
}
// 与固件 proto::ppm_to_int8 一致
static inline int8_t ub_ppm_to_int8(float ppm) {
  if (ppm != ppm) { // NaN
    return UB_PPM_INT8_INVALID;
  }
  if (ppm <= -127.0f) {
    return -127;
  }
  if (ppm >= 127.0f) {
    return 127;
  }
  return (int8_t)ub_round_i32(ppm);
}

// ---------------- 线格式结构 ----------------

#pragma pack(push, 1)

typedef struct {
  uint8_t delay;
  uint8_t only_restart_when_need : 1;
} UBRawDataRestart;

typedef struct {
  uint8_t duration;
} UBRawDataFind;

// 未在 UBDataParam 中公开的字段（expect_location_frequency / arg* /
// init_point_count / map_switch_window / locate_with_all_maps /
// deprecated_* / output_delay_ms）仍占据线上位置：读取时忽略，写入时由
// to_raw 填固件默认值。驱动无状态，因此每次写参数都会把这些隐藏字段恢复为默认值。
typedef struct {
  float expect_location_frequency;
  float expect_z;
  uint8_t _z_noise; // z_noise = 0.01 * _z_noise
  uint8_t smooth_window : 4;
  uint8_t _max_acceleration[3]; // max_acceleration = 0.02 * _max_acceleration
  struct {
    uint8_t tag_pos : 1;
    uint8_t anchor_packet : 1;
    uint8_t anchor_pos : 1;
    uint8_t anchor_link_data : 1;
    uint8_t anchor_signal : 1;
    uint8_t anchor_ddoa : 1;
    uint8_t tag_pos_even_error : 1;
    uint8_t anchor_link_status : 1;
  } output;
  uint8_t sniff_duty_cycle;
  uint8_t update_interval_max;
  uint16_t reset_interval;
  uint8_t min_sync_anchor_count;
  uint8_t min_sniff_duty_cycle;
  uint8_t arg0;
  uint8_t _arg1; // arg1 = _arg1 / 200
  uint8_t _arg2; // arg2 = _arg2 / 10
  uint8_t init_point_count;
  uint8_t map_switch_window;
  uint8_t locate_with_all_maps : 1;
  uint8_t deprecated_disable_anchor_data_output_with_result : 1;
  uint8_t : 0;
  uint8_t static_map_id;
  uint8_t output_delay_ms;
  uint8_t output_imu : 1;
  uint8_t output_attitude : 1;
  uint8_t output_dis : 1;
  uint8_t output_pdoa : 1;
  uint8_t : 0;
} UBRawDataParam;

typedef struct {
  uint8_t sniff_duty_cycle;
} UBRawDataRunTimeParam;

typedef struct {
  ub_local_time_us_t local_time_us;
  float pos[3];
  int16_t vel_cm[3];
  uint8_t pos_noise_cm[3];
  uint8_t vel_noise_cm[3];
  uint8_t map_id;
  uint8_t error_code : 4;
  uint8_t area_id : 4;
} UBRawDataLocationResult;

typedef struct {
  uint8_t uart : 1;
  uint8_t iic : 1;
  uint8_t uwb : 1;
  uint8_t ble : 1;
  uint8_t four_g : 1;
  uint8_t motor : 1;
  uint8_t bee : 1;
  uint8_t priority : 1;
  uint8_t key : 1;
  uint8_t motion : 1;
  uint8_t battery : 1;
  uint8_t battery_chargeable : 1;
  uint8_t : 1;
  uint8_t baro : 1;
  uint8_t imu : 1;
  uint8_t gnss : 1;
} UBRawDataPeripheralConfig;

typedef struct {
  uint32_t baudrate;
} UBRawDataUartInterfaceParam;

typedef struct {
  uint8_t addr : 7;
} UBRawDataIicInterfaceParam;

typedef struct {
  uint8_t reserved[2];
  uint8_t _max_rx_interval; // max_rx_interval_ms = 50 * _max_rx_interval
  uint16_t min_tx_interval_ms;
  uint16_t reserved2;
  uint16_t random_window_ms;
} UBRawDataUwbInterfaceParam;

typedef struct {
  float dis_frequency;
  uint8_t dis_slot_count_max;
  uint8_t dis_remain_slot_count;
} UBRawDataTwrParam;

typedef struct {
  uint8_t battery_percent : 7;
  uint8_t battery_charging : 1;
  uint8_t need_restart : 1;
  uint8_t reset_info_dirty : 1;
  uint8_t assert_info_dirty : 1;
  uint8_t restart_cnt : 3;
  uint8_t user_request_sleep : 1;
  uint8_t hardware_enabled_motion : 1;
  uint8_t hardware_enabled_uart : 1;
  uint8_t hardware_enabled_iic : 1;
  uint8_t hardware_enabled_uwb : 1;
  uint8_t hardware_enabled_ble : 1;
  uint8_t hardware_enabled_four_g : 1;
  uint8_t hardware_enabled_baro : 1;
  uint8_t hardware_enabled_imu : 1;
  uint8_t hardware_enabled_gnss : 1;
  uint8_t firmware_series;
  uint8_t firmware_version[4];
  uint8_t uid[UB_UID_SIZE];
  uint8_t _battery_voltage; // battery_voltage = _battery_voltage / 40.0
  uint8_t recent_rx_sts_error_occurred : 1;
  uint8_t recent_rx_data_error_occurred : 1;
  uint8_t recent_global_time_error_occurred : 1;
  uint8_t recent_peer_global_time_error_occurred : 1;
  uint8_t recent_peer_local_time_error_occurred : 1;
  uint8_t rtos_warn : 1;
  uint8_t mcu_warn : 1;
  uint8_t uwb_warn : 1;
} UBRawDataHeartbeat;

typedef struct {
  uint8_t reserved;
  uint8_t payload_size;
  uint8_t payload[UB_USER_DATA_PAYLOAD_MAX];
} UBRawDataUserData;

typedef struct {
  ub_local_time_us_t begin_time_us;
  uint8_t group_count : 4;
  uint8_t group_index : 4;
  uint8_t count;
  struct {
    ub_addr_t a0;
    ub_addr_t a1;
    int16_t ddoa_cm;
  } datas[UB_ANCHOR2TAG_DDOA_DATA_MAX];
} UBRawDataAnchor2TagDdoa;

typedef struct {
  ub_local_time_us_t begin_time_us;
  uint8_t reserved;
  uint8_t count : 4;
  uint8_t location_mode : 4;
  struct {
    ub_addr_t addr;
    uint8_t _rx_rssi;             // rx_rssi = -_rx_rssi * 0.5
    uint8_t _fp_rssi;             // fp_rssi = -_fp_rssi * 0.5
    int8_t uwb_clock_offset_ppm;  // 见 ub_ppm_from_int8
    int8_t mcu_clock_offset_ppm;  // 见 ub_ppm_from_int8
    uint8_t _rx_rate;             // rx_rate = _rx_rate / 255
  } datas[UB_ANCHOR2TAG_SIGNAL_DATA_MAX];
} UBRawDataAnchor2TagSignal;

typedef struct {
  ub_local_time_us_t begin_time_us;
  uint8_t reserved;
  uint8_t count : 4;
  uint8_t : 0;
  struct {
    ub_addr_t addr;
    int16_t dis_cm;
  } datas[UB_TAG2ANCHOR_DIS_DATA_MAX];
} UBRawDataTag2AnchorDis;

typedef struct {
  ub_local_time_us_t begin_time_us;
  uint8_t reserved;
  uint8_t count : 4;
  uint8_t : 0;
  struct {
    ub_addr_t addr;
    uint8_t antenna_type : 4;
    uint8_t pdoa_count : 4;
    int16_t _pdoas[UB_TAG2ANCHOR_PDOA_COUNT_MAX]; // pdoa = _pdoa / 10000
  } datas[UB_TAG2ANCHOR_PDOA_DATA_MAX];
} UBRawDataTag2AnchorPdoa;

typedef struct {
  ub_local_time_us_t local_time_us;
  float gyro[3];
  float acc[3];
} UBRawDataImu;

typedef struct {
  ub_local_time_us_t local_time_us;
  float q[4];
  float noise[3];
} UBRawDataAttitude;

typedef struct {
  int32_t z_cm;
  uint8_t z_std_cm;
  uint8_t timeout;
} UBRawDataZMeasurement;

typedef struct {
  uint8_t sleep : 1;
  uint8_t : 0;
} UBRawDataStateControl;

typedef struct {
  uint8_t map_id;
  uint8_t timeout;
} UBRawDataMapMeasurement;

typedef struct {
  uint8_t run : 1;
  uint8_t timeout : 7;
  uint64_t ttl : 6;
  uint64_t : 2;
  uint64_t time : 56;
  ub_addr_t src;
} UBRawDataGlobalTimeStatus;

typedef struct {
  ub_local_time_us_t local_time;
  ub_addr_t src;
  int32_t pos_cm[3];
  uint8_t is_local_pos : 1;
  uint8_t : 0;
  uint8_t map_id;
  int16_t relative_map_z_cm; // map_z_cm = pos_cm[2] + relative_map_z_cm
} UBRawDataAnchorPos;

#pragma pack(pop)

// ---------------- 转换器 ----------------

static inline void ub_data_restart_from_raw(const UBRawDataRestart *raw,
                                            UBDataRestart *data) {
  data->delay = raw->delay;
  data->only_restart_when_need = raw->only_restart_when_need != 0;
}
static inline void ub_data_restart_to_raw(const UBDataRestart *data, void *buf,
                                          int *size) {
  UBRawDataRestart *raw = (UBRawDataRestart *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->delay = data->delay;
  raw->only_restart_when_need = data->only_restart_when_need ? 1 : 0;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_find_from_raw(const UBRawDataFind *raw,
                                         UBDataFind *data) {
  data->duration = raw->duration;
}
static inline void ub_data_find_to_raw(const UBDataFind *data, void *buf,
                                       int *size) {
  UBRawDataFind *raw = (UBRawDataFind *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->duration = data->duration;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_param_from_raw(const UBRawDataParam *raw,
                                          UBDataParam *data) {
  data->expect_z = raw->expect_z;
  data->z_noise = raw->_z_noise * 0.01f;
  data->smooth_window = raw->smooth_window;
  for (int i = 0; i < 3; ++i) {
    data->max_acceleration[i] = raw->_max_acceleration[i] * 0.02f;
  }
  data->output.tag_pos = raw->output.tag_pos != 0;
  data->output.anchor_packet = raw->output.anchor_packet != 0;
  data->output.anchor_pos = raw->output.anchor_pos != 0;
  data->output.anchor_link_data = raw->output.anchor_link_data != 0;
  data->output.anchor_signal = raw->output.anchor_signal != 0;
  data->output.anchor_ddoa = raw->output.anchor_ddoa != 0;
  data->output.tag_pos_even_error = raw->output.tag_pos_even_error != 0;
  data->output.anchor_link_status = raw->output.anchor_link_status != 0;
  data->output.imu = raw->output_imu != 0;
  data->output.attitude = raw->output_attitude != 0;
  data->output.dis = raw->output_dis != 0;
  data->output.pdoa = raw->output_pdoa != 0;
  data->sniff_duty_cycle = raw->sniff_duty_cycle;
  data->update_interval_max = raw->update_interval_max;
  data->reset_interval = raw->reset_interval;
  data->min_sync_anchor_count = raw->min_sync_anchor_count;
  data->min_sniff_duty_cycle = raw->min_sniff_duty_cycle;
  data->static_map_id = raw->static_map_id;
}
static inline void ub_data_param_to_raw(const UBDataParam *data, void *buf,
                                        int *size) {
  UBRawDataParam *raw = (UBRawDataParam *)buf;
  memset(raw, 0, sizeof(*raw));
  // 隐藏字段：arg*/init_point_count/map_switch_window 为 0 时固件自动取默认值；
  // 下面两个没有该规则，0 会被固件钳成 0.1Hz / 关闭，必须显式给默认值
  raw->expect_location_frequency = 1.0f;
  raw->locate_with_all_maps = 1;
  raw->expect_z = data->expect_z;
  raw->_z_noise = (uint8_t)ub_round_i32(data->z_noise / 0.01f);
  raw->smooth_window = data->smooth_window & 0x0F;
  for (int i = 0; i < 3; ++i) {
    raw->_max_acceleration[i] =
        (uint8_t)ub_round_i32(data->max_acceleration[i] / 0.02f);
  }
  raw->output.tag_pos = data->output.tag_pos ? 1 : 0;
  raw->output.anchor_packet = data->output.anchor_packet ? 1 : 0;
  raw->output.anchor_pos = data->output.anchor_pos ? 1 : 0;
  raw->output.anchor_link_data = data->output.anchor_link_data ? 1 : 0;
  raw->output.anchor_signal = data->output.anchor_signal ? 1 : 0;
  raw->output.anchor_ddoa = data->output.anchor_ddoa ? 1 : 0;
  raw->output.tag_pos_even_error = data->output.tag_pos_even_error ? 1 : 0;
  raw->output.anchor_link_status = data->output.anchor_link_status ? 1 : 0;
  raw->output_imu = data->output.imu ? 1 : 0;
  raw->output_attitude = data->output.attitude ? 1 : 0;
  raw->output_dis = data->output.dis ? 1 : 0;
  raw->output_pdoa = data->output.pdoa ? 1 : 0;
  raw->sniff_duty_cycle = data->sniff_duty_cycle;
  raw->update_interval_max = data->update_interval_max;
  raw->reset_interval = data->reset_interval;
  raw->min_sync_anchor_count = data->min_sync_anchor_count;
  raw->min_sniff_duty_cycle = data->min_sniff_duty_cycle;
  raw->static_map_id = data->static_map_id;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_run_time_param_from_raw(
    const UBRawDataRunTimeParam *raw, UBDataRunTimeParam *data) {
  data->sniff_duty_cycle = raw->sniff_duty_cycle;
}
static inline void ub_data_run_time_param_to_raw(const UBDataRunTimeParam *data,
                                                 void *buf, int *size) {
  UBRawDataRunTimeParam *raw = (UBRawDataRunTimeParam *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->sniff_duty_cycle = data->sniff_duty_cycle;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_location_result_from_raw(
    const UBRawDataLocationResult *raw, UBDataLocationResult *data) {
  data->local_time_us = raw->local_time_us;
  for (int i = 0; i < 3; ++i) {
    data->pos[i] = raw->pos[i];
    data->vel[i] = raw->vel_cm[i] * 0.01f;
    data->pos_noise[i] = raw->pos_noise_cm[i] * 0.01f;
    data->vel_noise[i] = raw->vel_noise_cm[i] * 0.01f;
  }
  data->map_id = raw->map_id;
  data->error_code = raw->error_code;
  data->area_id = raw->area_id;
}
static inline void ub_data_location_result_to_raw(
    const UBDataLocationResult *data, void *buf, int *size) {
  UBRawDataLocationResult *raw = (UBRawDataLocationResult *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->local_time_us = data->local_time_us;
  for (int i = 0; i < 3; ++i) {
    raw->pos[i] = data->pos[i];
    raw->vel_cm[i] = (int16_t)ub_round_i32(data->vel[i] * 100.0f);
    raw->pos_noise_cm[i] = (uint8_t)ub_round_i32(data->pos_noise[i] * 100.0f);
    raw->vel_noise_cm[i] = (uint8_t)ub_round_i32(data->vel_noise[i] * 100.0f);
  }
  raw->map_id = data->map_id;
  raw->error_code = data->error_code & 0x0F;
  raw->area_id = data->area_id & 0x0F;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_peripheral_config_from_raw(
    const UBRawDataPeripheralConfig *raw, UBDataPeripheralConfig *data) {
  data->uart = raw->uart != 0;
  data->iic = raw->iic != 0;
  data->uwb = raw->uwb != 0;
  data->ble = raw->ble != 0;
  data->four_g = raw->four_g != 0;
  data->priority = raw->priority != 0;
  data->motor = raw->motor != 0;
  data->bee = raw->bee != 0;
  data->key = raw->key != 0;
  data->motion = raw->motion != 0;
  data->battery = raw->battery != 0;
  data->battery_chargeable = raw->battery_chargeable != 0;
  data->baro = raw->baro != 0;
  data->imu = raw->imu != 0;
  data->gnss = raw->gnss != 0;
}
static inline void ub_data_peripheral_config_to_raw(
    const UBDataPeripheralConfig *data, void *buf, int *size) {
  UBRawDataPeripheralConfig *raw = (UBRawDataPeripheralConfig *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->uart = data->uart ? 1 : 0;
  raw->iic = data->iic ? 1 : 0;
  raw->uwb = data->uwb ? 1 : 0;
  raw->ble = data->ble ? 1 : 0;
  raw->four_g = data->four_g ? 1 : 0;
  raw->priority = data->priority ? 1 : 0;
  raw->motor = data->motor ? 1 : 0;
  raw->bee = data->bee ? 1 : 0;
  raw->key = data->key ? 1 : 0;
  raw->motion = data->motion ? 1 : 0;
  raw->battery = data->battery ? 1 : 0;
  raw->battery_chargeable = data->battery_chargeable ? 1 : 0;
  raw->baro = data->baro ? 1 : 0;
  raw->imu = data->imu ? 1 : 0;
  raw->gnss = data->gnss ? 1 : 0;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_uart_interface_param_from_raw(
    const UBRawDataUartInterfaceParam *raw, UBDataUartInterfaceParam *data) {
  data->baudrate = raw->baudrate;
}
static inline void ub_data_uart_interface_param_to_raw(
    const UBDataUartInterfaceParam *data, void *buf, int *size) {
  UBRawDataUartInterfaceParam *raw = (UBRawDataUartInterfaceParam *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->baudrate = data->baudrate;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_iic_interface_param_from_raw(
    const UBRawDataIicInterfaceParam *raw, UBDataIicInterfaceParam *data) {
  data->addr = raw->addr;
}
static inline void ub_data_iic_interface_param_to_raw(
    const UBDataIicInterfaceParam *data, void *buf, int *size) {
  UBRawDataIicInterfaceParam *raw = (UBRawDataIicInterfaceParam *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->addr = data->addr & 0x7F;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_uwb_interface_param_from_raw(
    const UBRawDataUwbInterfaceParam *raw, UBDataUwbInterfaceParam *data) {
  data->random_window_ms = raw->random_window_ms;
  data->max_rx_interval_ms = (uint16_t)(raw->_max_rx_interval * 50);
  data->min_tx_interval_ms = raw->min_tx_interval_ms;
}
static inline void ub_data_uwb_interface_param_to_raw(
    const UBDataUwbInterfaceParam *data, void *buf, int *size) {
  UBRawDataUwbInterfaceParam *raw = (UBRawDataUwbInterfaceParam *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->random_window_ms = data->random_window_ms;
  raw->_max_rx_interval = (uint8_t)(data->max_rx_interval_ms / 50);
  raw->min_tx_interval_ms = data->min_tx_interval_ms;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_twr_param_from_raw(const UBRawDataTwrParam *raw,
                                              UBDataTwrParam *data) {
  data->dis_frequency = raw->dis_frequency;
  data->dis_slot_count_max = raw->dis_slot_count_max;
  data->dis_remain_slot_count = raw->dis_remain_slot_count;
}
static inline void ub_data_twr_param_to_raw(const UBDataTwrParam *data,
                                            void *buf, int *size) {
  UBRawDataTwrParam *raw = (UBRawDataTwrParam *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->dis_frequency = data->dis_frequency;
  raw->dis_slot_count_max = data->dis_slot_count_max;
  raw->dis_remain_slot_count = data->dis_remain_slot_count;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_heartbeat_from_raw(const UBRawDataHeartbeat *raw,
                                              UBDataHeartbeat *data) {
  data->battery_percent = raw->battery_percent;
  data->battery_charging = raw->battery_charging != 0;
  data->need_restart = raw->need_restart != 0;
  data->reset_info_dirty = raw->reset_info_dirty != 0;
  data->assert_info_dirty = raw->assert_info_dirty != 0;
  data->restart_cnt = raw->restart_cnt;
  data->user_request_sleep = raw->user_request_sleep != 0;
  data->hardware_enabled_motion = raw->hardware_enabled_motion != 0;
  data->hardware_enabled_uart = raw->hardware_enabled_uart != 0;
  data->hardware_enabled_iic = raw->hardware_enabled_iic != 0;
  data->hardware_enabled_uwb = raw->hardware_enabled_uwb != 0;
  data->hardware_enabled_ble = raw->hardware_enabled_ble != 0;
  data->hardware_enabled_four_g = raw->hardware_enabled_four_g != 0;
  data->hardware_enabled_baro = raw->hardware_enabled_baro != 0;
  data->hardware_enabled_imu = raw->hardware_enabled_imu != 0;
  data->hardware_enabled_gnss = raw->hardware_enabled_gnss != 0;
  data->firmware_series = raw->firmware_series;
  memcpy(data->firmware_version, raw->firmware_version,
         sizeof(data->firmware_version));
  memcpy(data->uid, raw->uid, sizeof(data->uid));
  data->battery_voltage = raw->_battery_voltage / 40.0f;
  data->recent_rx_sts_error_occurred = raw->recent_rx_sts_error_occurred != 0;
  data->recent_rx_data_error_occurred =
      raw->recent_rx_data_error_occurred != 0;
  data->recent_global_time_error_occurred =
      raw->recent_global_time_error_occurred != 0;
  data->recent_peer_global_time_error_occurred =
      raw->recent_peer_global_time_error_occurred != 0;
  data->recent_peer_local_time_error_occurred =
      raw->recent_peer_local_time_error_occurred != 0;
  data->rtos_warn = raw->rtos_warn != 0;
  data->mcu_warn = raw->mcu_warn != 0;
  data->uwb_warn = raw->uwb_warn != 0;
}
static inline void ub_data_heartbeat_to_raw(const UBDataHeartbeat *data,
                                            void *buf, int *size) {
  UBRawDataHeartbeat *raw = (UBRawDataHeartbeat *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->battery_percent = data->battery_percent & 0x7F;
  raw->battery_charging = data->battery_charging ? 1 : 0;
  raw->need_restart = data->need_restart ? 1 : 0;
  raw->reset_info_dirty = data->reset_info_dirty ? 1 : 0;
  raw->assert_info_dirty = data->assert_info_dirty ? 1 : 0;
  raw->restart_cnt = data->restart_cnt & 0x07;
  raw->user_request_sleep = data->user_request_sleep ? 1 : 0;
  raw->hardware_enabled_motion = data->hardware_enabled_motion ? 1 : 0;
  raw->hardware_enabled_uart = data->hardware_enabled_uart ? 1 : 0;
  raw->hardware_enabled_iic = data->hardware_enabled_iic ? 1 : 0;
  raw->hardware_enabled_uwb = data->hardware_enabled_uwb ? 1 : 0;
  raw->hardware_enabled_ble = data->hardware_enabled_ble ? 1 : 0;
  raw->hardware_enabled_four_g = data->hardware_enabled_four_g ? 1 : 0;
  raw->hardware_enabled_baro = data->hardware_enabled_baro ? 1 : 0;
  raw->hardware_enabled_imu = data->hardware_enabled_imu ? 1 : 0;
  raw->hardware_enabled_gnss = data->hardware_enabled_gnss ? 1 : 0;
  raw->firmware_series = data->firmware_series;
  memcpy(raw->firmware_version, data->firmware_version,
         sizeof(raw->firmware_version));
  memcpy(raw->uid, data->uid, sizeof(raw->uid));
  raw->_battery_voltage =
      (uint8_t)ub_round_i32(data->battery_voltage * 40.0f);
  raw->recent_rx_sts_error_occurred = data->recent_rx_sts_error_occurred ? 1 : 0;
  raw->recent_rx_data_error_occurred =
      data->recent_rx_data_error_occurred ? 1 : 0;
  raw->recent_global_time_error_occurred =
      data->recent_global_time_error_occurred ? 1 : 0;
  raw->recent_peer_global_time_error_occurred =
      data->recent_peer_global_time_error_occurred ? 1 : 0;
  raw->recent_peer_local_time_error_occurred =
      data->recent_peer_local_time_error_occurred ? 1 : 0;
  raw->rtos_warn = data->rtos_warn ? 1 : 0;
  raw->mcu_warn = data->mcu_warn ? 1 : 0;
  raw->uwb_warn = data->uwb_warn ? 1 : 0;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_user_data_from_raw(const UBRawDataUserData *raw,
                                              UBDataUserData *data) {
  data->payload_size =
      (uint8_t)ub_min_int(raw->payload_size, UB_USER_DATA_PAYLOAD_MAX);
  memcpy(data->payload, raw->payload, data->payload_size);
}
static inline void ub_data_user_data_to_raw(const UBDataUserData *data,
                                            void *buf, int *size) {
  UBRawDataUserData *raw = (UBRawDataUserData *)buf;
  memset(raw, 0, sizeof(*raw));
  int payload_size = ub_min_int(data->payload_size, UB_USER_DATA_PAYLOAD_MAX);
  raw->payload_size = (uint8_t)payload_size;
  memcpy(raw->payload, data->payload, payload_size);
  // 变长：只发送有效载荷
  *size = (int)offsetof(UBRawDataUserData, payload) + payload_size;
}

static inline void ub_data_anchor2tag_ddoa_from_raw(
    const UBRawDataAnchor2TagDdoa *raw, UBDataAnchor2TagDdoa *data) {
  data->begin_time_us = raw->begin_time_us;
  data->group_count = raw->group_count;
  data->group_index = raw->group_index;
  data->count = (uint8_t)ub_min_int(raw->count, UB_ANCHOR2TAG_DDOA_DATA_MAX);
  for (int i = 0; i < data->count; ++i) {
    data->datas[i].a0 = raw->datas[i].a0;
    data->datas[i].a1 = raw->datas[i].a1;
    data->datas[i].ddoa = raw->datas[i].ddoa_cm * 0.01f;
  }
}
static inline void ub_data_anchor2tag_ddoa_to_raw(
    const UBDataAnchor2TagDdoa *data, void *buf, int *size) {
  UBRawDataAnchor2TagDdoa *raw = (UBRawDataAnchor2TagDdoa *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->begin_time_us = data->begin_time_us;
  raw->group_count = data->group_count & 0x0F;
  raw->group_index = data->group_index & 0x0F;
  int count = ub_min_int(data->count, UB_ANCHOR2TAG_DDOA_DATA_MAX);
  raw->count = (uint8_t)count;
  for (int i = 0; i < count; ++i) {
    raw->datas[i].a0 = data->datas[i].a0;
    raw->datas[i].a1 = data->datas[i].a1;
    raw->datas[i].ddoa_cm = (int16_t)ub_round_i32(data->datas[i].ddoa * 100.0f);
  }
  // 变长：只发送有效元素
  *size = (int)offsetof(UBRawDataAnchor2TagDdoa, datas) +
          count * (int)sizeof(raw->datas[0]);
}

static inline void ub_data_anchor2tag_signal_from_raw(
    const UBRawDataAnchor2TagSignal *raw, UBDataAnchor2TagSignal *data) {
  data->begin_time_us = raw->begin_time_us;
  data->count =
      (uint8_t)ub_min_int(raw->count, UB_ANCHOR2TAG_SIGNAL_DATA_MAX);
  data->location_mode = raw->location_mode;
  for (int i = 0; i < data->count; ++i) {
    data->datas[i].addr = raw->datas[i].addr;
    data->datas[i].rx_rssi = raw->datas[i]._rx_rssi * -0.5f;
    data->datas[i].fp_rssi = raw->datas[i]._fp_rssi * -0.5f;
    data->datas[i].uwb_clock_offset_ppm =
        ub_ppm_from_int8(raw->datas[i].uwb_clock_offset_ppm);
    data->datas[i].mcu_clock_offset_ppm =
        ub_ppm_from_int8(raw->datas[i].mcu_clock_offset_ppm);
    data->datas[i].rx_rate = raw->datas[i]._rx_rate / 255.0f;
  }
}
static inline void ub_data_anchor2tag_signal_to_raw(
    const UBDataAnchor2TagSignal *data, void *buf, int *size) {
  UBRawDataAnchor2TagSignal *raw = (UBRawDataAnchor2TagSignal *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->begin_time_us = data->begin_time_us;
  int count = ub_min_int(data->count, UB_ANCHOR2TAG_SIGNAL_DATA_MAX);
  raw->count = count & 0x0F;
  raw->location_mode = data->location_mode & 0x0F;
  for (int i = 0; i < count; ++i) {
    raw->datas[i].addr = data->datas[i].addr;
    raw->datas[i]._rx_rssi =
        (uint8_t)ub_round_i32(data->datas[i].rx_rssi / -0.5f);
    raw->datas[i]._fp_rssi =
        (uint8_t)ub_round_i32(data->datas[i].fp_rssi / -0.5f);
    raw->datas[i].uwb_clock_offset_ppm =
        ub_ppm_to_int8(data->datas[i].uwb_clock_offset_ppm);
    raw->datas[i].mcu_clock_offset_ppm =
        ub_ppm_to_int8(data->datas[i].mcu_clock_offset_ppm);
    raw->datas[i]._rx_rate =
        (uint8_t)ub_round_i32(data->datas[i].rx_rate * 255.0f);
  }
  // 变长：只发送有效元素
  *size = (int)offsetof(UBRawDataAnchor2TagSignal, datas) +
          count * (int)sizeof(raw->datas[0]);
}

static inline void ub_data_tag2anchor_dis_from_raw(
    const UBRawDataTag2AnchorDis *raw, UBDataTag2AnchorDis *data) {
  data->begin_time_us = raw->begin_time_us;
  data->count = (uint8_t)ub_min_int(raw->count, UB_TAG2ANCHOR_DIS_DATA_MAX);
  for (int i = 0; i < data->count; ++i) {
    data->datas[i].addr = raw->datas[i].addr;
    data->datas[i].dis = raw->datas[i].dis_cm * 0.01f;
  }
}
static inline void ub_data_tag2anchor_dis_to_raw(
    const UBDataTag2AnchorDis *data, void *buf, int *size) {
  UBRawDataTag2AnchorDis *raw = (UBRawDataTag2AnchorDis *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->begin_time_us = data->begin_time_us;
  int count = ub_min_int(data->count, UB_TAG2ANCHOR_DIS_DATA_MAX);
  raw->count = count & 0x0F;
  for (int i = 0; i < count; ++i) {
    raw->datas[i].addr = data->datas[i].addr;
    raw->datas[i].dis_cm = (int16_t)ub_round_i32(data->datas[i].dis * 100.0f);
  }
  // 变长：只发送有效元素
  *size = (int)offsetof(UBRawDataTag2AnchorDis, datas) +
          count * (int)sizeof(raw->datas[0]);
}

static inline void ub_data_tag2anchor_pdoa_from_raw(
    const UBRawDataTag2AnchorPdoa *raw, UBDataTag2AnchorPdoa *data) {
  data->begin_time_us = raw->begin_time_us;
  data->count = (uint8_t)ub_min_int(raw->count, UB_TAG2ANCHOR_PDOA_DATA_MAX);
  for (int i = 0; i < data->count; ++i) {
    data->datas[i].addr = raw->datas[i].addr;
    data->datas[i].antenna_type = raw->datas[i].antenna_type;
    data->datas[i].pdoa_count = (uint8_t)ub_min_int(
        raw->datas[i].pdoa_count, UB_TAG2ANCHOR_PDOA_COUNT_MAX);
    for (int j = 0; j < data->datas[i].pdoa_count; ++j) {
      data->datas[i].pdoas[j] = raw->datas[i]._pdoas[j] / 10000.0f;
    }
  }
}
static inline void ub_data_tag2anchor_pdoa_to_raw(
    const UBDataTag2AnchorPdoa *data, void *buf, int *size) {
  UBRawDataTag2AnchorPdoa *raw = (UBRawDataTag2AnchorPdoa *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->begin_time_us = data->begin_time_us;
  int count = ub_min_int(data->count, UB_TAG2ANCHOR_PDOA_DATA_MAX);
  raw->count = count & 0x0F;
  for (int i = 0; i < count; ++i) {
    raw->datas[i].addr = data->datas[i].addr;
    raw->datas[i].antenna_type = data->datas[i].antenna_type & 0x0F;
    int pdoa_count =
        ub_min_int(data->datas[i].pdoa_count, UB_TAG2ANCHOR_PDOA_COUNT_MAX);
    raw->datas[i].pdoa_count = pdoa_count & 0x0F;
    for (int j = 0; j < pdoa_count; ++j) {
      raw->datas[i]._pdoas[j] =
          (int16_t)ub_round_i32(data->datas[i].pdoas[j] * 10000.0f);
    }
  }
  // 变长：只发送有效元素
  *size = (int)offsetof(UBRawDataTag2AnchorPdoa, datas) +
          count * (int)sizeof(raw->datas[0]);
}

static inline void ub_data_imu_from_raw(const UBRawDataImu *raw,
                                        UBDataImu *data) {
  data->local_time_us = raw->local_time_us;
  for (int i = 0; i < 3; ++i) {
    data->gyro[i] = raw->gyro[i];
    data->acc[i] = raw->acc[i];
  }
}
static inline void ub_data_imu_to_raw(const UBDataImu *data, void *buf,
                                      int *size) {
  UBRawDataImu *raw = (UBRawDataImu *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->local_time_us = data->local_time_us;
  for (int i = 0; i < 3; ++i) {
    raw->gyro[i] = data->gyro[i];
    raw->acc[i] = data->acc[i];
  }
  *size = (int)sizeof(*raw);
}

static inline void ub_data_attitude_from_raw(const UBRawDataAttitude *raw,
                                             UBDataAttitude *data) {
  data->local_time_us = raw->local_time_us;
  for (int i = 0; i < 4; ++i) {
    data->q[i] = raw->q[i];
  }
  for (int i = 0; i < 3; ++i) {
    data->noise[i] = raw->noise[i];
  }
}
static inline void ub_data_attitude_to_raw(const UBDataAttitude *data,
                                           void *buf, int *size) {
  UBRawDataAttitude *raw = (UBRawDataAttitude *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->local_time_us = data->local_time_us;
  for (int i = 0; i < 4; ++i) {
    raw->q[i] = data->q[i];
  }
  for (int i = 0; i < 3; ++i) {
    raw->noise[i] = data->noise[i];
  }
  *size = (int)sizeof(*raw);
}

static inline void ub_data_z_measurement_from_raw(
    const UBRawDataZMeasurement *raw, UBDataZMeasurement *data) {
  data->z = raw->z_cm * 0.01f;
  data->z_std = raw->z_std_cm * 0.01f;
  data->timeout = raw->timeout;
}
static inline void ub_data_z_measurement_to_raw(const UBDataZMeasurement *data,
                                                void *buf, int *size) {
  UBRawDataZMeasurement *raw = (UBRawDataZMeasurement *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->z_cm = ub_round_i32(data->z * 100.0f);
  raw->z_std_cm = (uint8_t)ub_round_i32(data->z_std * 100.0f);
  raw->timeout = data->timeout;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_state_control_from_raw(
    const UBRawDataStateControl *raw, UBDataStateControl *data) {
  data->sleep = raw->sleep != 0;
}
static inline void ub_data_state_control_to_raw(const UBDataStateControl *data,
                                                void *buf, int *size) {
  UBRawDataStateControl *raw = (UBRawDataStateControl *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->sleep = data->sleep ? 1 : 0;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_map_measurement_from_raw(
    const UBRawDataMapMeasurement *raw, UBDataMapMeasurement *data) {
  data->map_id = raw->map_id;
  data->timeout = raw->timeout;
}
static inline void ub_data_map_measurement_to_raw(
    const UBDataMapMeasurement *data, void *buf, int *size) {
  UBRawDataMapMeasurement *raw = (UBRawDataMapMeasurement *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->map_id = data->map_id;
  raw->timeout = data->timeout;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_global_time_status_from_raw(
    const UBRawDataGlobalTimeStatus *raw, UBDataGlobalTimeStatus *data) {
  data->run = raw->run != 0;
  data->timeout = raw->timeout;
  data->ttl = (uint8_t)raw->ttl;
  data->time_us = raw->time;
  data->src = raw->src;
}
static inline void ub_data_global_time_status_to_raw(
    const UBDataGlobalTimeStatus *data, void *buf, int *size) {
  UBRawDataGlobalTimeStatus *raw = (UBRawDataGlobalTimeStatus *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->run = data->run ? 1 : 0;
  raw->timeout = data->timeout & 0x7F;
  raw->ttl = data->ttl & 0x3F;
  raw->time = data->time_us & 0xFFFFFFFFFFFFFFULL; // 56 位
  raw->src = data->src;
  *size = (int)sizeof(*raw);
}

static inline void ub_data_anchor_pos_from_raw(const UBRawDataAnchorPos *raw,
                                               UBDataAnchorPos *data) {
  data->local_time_us = raw->local_time;
  data->src = raw->src;
  for (int i = 0; i < 3; ++i) {
    data->pos[i] = raw->pos_cm[i] * 0.01f;
  }
  data->is_local_pos = raw->is_local_pos != 0;
  data->map_id = raw->map_id;
  data->relative_map_z = raw->relative_map_z_cm * 0.01f;
}
static inline void ub_data_anchor_pos_to_raw(const UBDataAnchorPos *data,
                                             void *buf, int *size) {
  UBRawDataAnchorPos *raw = (UBRawDataAnchorPos *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->local_time = data->local_time_us;
  raw->src = data->src;
  for (int i = 0; i < 3; ++i) {
    raw->pos_cm[i] = ub_round_i32(data->pos[i] * 100.0f);
  }
  raw->is_local_pos = data->is_local_pos ? 1 : 0;
  raw->map_id = data->map_id;
  raw->relative_map_z_cm =
      (int16_t)ub_round_i32(data->relative_map_z * 100.0f);
  *size = (int)sizeof(*raw);
}

#ifdef __cplusplus
}
#endif
