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
#include <stddef.h>
#include <string.h>

// ---------------- 转换辅助 ----------------

// 四舍五入，避免引入 <math.h>（嵌入式侧无需链接 libm）
static inline int32_t ub_round_i32(float v) {
  return (int32_t)(v < 0.0f ? v - 0.5f : v + 0.5f);
}

static inline int ub_min_int(int a, int b) { return a < b ? a : b; }

// ---------------- 线格式结构 ----------------

#pragma pack(push, 1)

typedef struct {
  uint8_t delay;
  uint8_t only_restart_when_need : 1;
} UBRawDataRestart;

typedef struct {
  uint8_t duration;
} UBRawDataFind;

typedef struct {
  uint8_t reserved[4];
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
  uint8_t reserved;
  uint8_t anchor_count : 4;
  uint8_t : 0;
  struct {
    ub_addr_t addr;
    uint8_t _rx_rssi; // rx_rssi = _rx_rssi / -2
    uint8_t _rx_rate; // rx_rate = _rx_rate / 255
  } anchors[UB_LOCATION_RESULT_ANCHOR_MAX];
} UBRawDataLocationResult;

typedef struct {
  uint8_t uart : 1;
  uint8_t iic : 1;
  uint8_t uwb : 1;
  uint8_t ble : 1;
  uint8_t : 0;
} UBRawDataInterfaceParam;

typedef struct {
  uint32_t baudrate;
} UBRawDataUartInterfaceParam;

typedef struct {
  uint8_t addr : 7;
} UBRawDataIicInterfaceParam;

typedef struct {
  uint8_t reserved;
  uint8_t _random_window;   // random_window = 0.01 * _random_window
  uint8_t _max_rx_interval; // max_rx_interval = 0.05 * _max_rx_interval
} UBRawDataUwbInterfaceParam;

typedef struct {
  uint8_t company_id[2];
  uint8_t data_type;
  uint8_t _random_window; // random_window = 0.01 * _random_window
  uint8_t send_count : 3;
} UBRawDataBleInterfaceParam;

typedef struct {
  uint8_t battery_percent : 7;
  uint8_t battery_charging : 1;
  uint8_t need_restart : 1;
  uint8_t reset_info_dirty : 1;
  uint8_t assert_info_dirty : 1;
  uint8_t restart_cnt : 3;
  uint8_t is_sleeping : 1;
  uint8_t : 0;
  uint8_t hardware_enabled_uart : 1;
  uint8_t hardware_enabled_iic : 1;
  uint8_t hardware_enabled_uwb : 1;
  uint8_t hardware_enabled_ble : 1;
  uint8_t : 0;
  uint8_t firmware_series;
  uint8_t firmware_version[4];
  uint8_t uid[UB_UID_SIZE];
  uint8_t _battery_voltage; // battery_voltage = _battery_voltage / 40.0
} UBRawDataHeartbeat;

typedef struct {
  uint8_t reserved;
  uint8_t payload_size;
  uint8_t payload[UB_USER_DATA_PAYLOAD_MAX];
} UBRawDataUserData;

typedef struct {
  ub_local_time_us_t local_time_us;
  uint8_t count : 4;
  uint8_t area_id : 4;
  uint8_t reserved;
  struct {
    ub_addr_t addr;
    int16_t _fp_index; // fp_index = _fp_index * 0.1
    int8_t fp_to_peak;
    uint8_t _mc;                    // mc = _mc * 0.01
    uint8_t _rx_rssi;               // rx_rssi = -_rx_rssi * 0.5
    uint8_t _fp_rssi;               // fp_rssi = -_fp_rssi * 0.5
    int16_t _uwb_clock_offset_ppm;  // = _uwb_clock_offset_ppm * 0.01
    int16_t _mcu_clock_offset_ppm;  // = _mcu_clock_offset_ppm * 0.01
    uint8_t _rx_rate;               // rx_rate = _rx_rate / 255
  } datas[UB_ANCHOR_SIGNAL_DATA_MAX];
} UBRawDataAnchorSignal;

typedef struct {
  ub_local_time_us_t local_time_us;
  uint8_t reserved;
  uint8_t count;
  struct {
    ub_addr_t a0;
    ub_addr_t a1;
    int16_t ddoa_cm; // ddoa = dis_tag_to_a1 - dis_tag_to_a0
  } datas[UB_ANCHOR_DDOAS_DATA_MAX];
} UBRawDataAnchorDdoas;

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
  data->sniff_duty_cycle = raw->sniff_duty_cycle;
  data->update_interval_max = raw->update_interval_max;
  data->reset_interval = raw->reset_interval;
}
static inline void ub_data_param_to_raw(const UBDataParam *data, void *buf,
                                        int *size) {
  UBRawDataParam *raw = (UBRawDataParam *)buf;
  memset(raw, 0, sizeof(*raw));
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
  raw->sniff_duty_cycle = data->sniff_duty_cycle;
  raw->update_interval_max = data->update_interval_max;
  raw->reset_interval = data->reset_interval;
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
  // anchor_count 为 4 位（最大 15），而数组仅 9 个元素，必须钳位以防越界
  data->anchor_count =
      (uint8_t)ub_min_int(raw->anchor_count, UB_LOCATION_RESULT_ANCHOR_MAX);
  for (int i = 0; i < data->anchor_count; ++i) {
    data->anchors[i].addr = raw->anchors[i].addr;
    data->anchors[i].rx_rssi = raw->anchors[i]._rx_rssi / -2.0f;
    data->anchors[i].rx_rate = raw->anchors[i]._rx_rate / 255.0f;
  }
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
  int count = ub_min_int(data->anchor_count, UB_LOCATION_RESULT_ANCHOR_MAX);
  raw->anchor_count = count & 0x0F;
  for (int i = 0; i < count; ++i) {
    raw->anchors[i].addr = data->anchors[i].addr;
    raw->anchors[i]._rx_rssi =
        (uint8_t)ub_round_i32(data->anchors[i].rx_rssi * -2.0f);
    raw->anchors[i]._rx_rate =
        (uint8_t)ub_round_i32(data->anchors[i].rx_rate * 255.0f);
  }
  // 变长：只发送有效的 anchor
  *size = (int)offsetof(UBRawDataLocationResult, anchors) +
          count * (int)sizeof(raw->anchors[0]);
}

static inline void ub_data_interface_param_from_raw(
    const UBRawDataInterfaceParam *raw, UBDataInterfaceParam *data) {
  data->uart = raw->uart != 0;
  data->iic = raw->iic != 0;
  data->uwb = raw->uwb != 0;
  data->ble = raw->ble != 0;
}
static inline void ub_data_interface_param_to_raw(
    const UBDataInterfaceParam *data, void *buf, int *size) {
  UBRawDataInterfaceParam *raw = (UBRawDataInterfaceParam *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->uart = data->uart ? 1 : 0;
  raw->iic = data->iic ? 1 : 0;
  raw->uwb = data->uwb ? 1 : 0;
  raw->ble = data->ble ? 1 : 0;
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
  data->random_window = raw->_random_window * 0.01f;
  data->max_rx_interval = raw->_max_rx_interval * 0.05f;
}
static inline void ub_data_uwb_interface_param_to_raw(
    const UBDataUwbInterfaceParam *data, void *buf, int *size) {
  UBRawDataUwbInterfaceParam *raw = (UBRawDataUwbInterfaceParam *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->_random_window = (uint8_t)ub_round_i32(data->random_window / 0.01f);
  raw->_max_rx_interval = (uint8_t)ub_round_i32(data->max_rx_interval / 0.05f);
  *size = (int)sizeof(*raw);
}

static inline void ub_data_ble_interface_param_from_raw(
    const UBRawDataBleInterfaceParam *raw, UBDataBleInterfaceParam *data) {
  data->company_id[0] = raw->company_id[0];
  data->company_id[1] = raw->company_id[1];
  data->data_type = raw->data_type;
  data->random_window = raw->_random_window * 0.01f;
  data->send_count = raw->send_count;
}
static inline void ub_data_ble_interface_param_to_raw(
    const UBDataBleInterfaceParam *data, void *buf, int *size) {
  UBRawDataBleInterfaceParam *raw = (UBRawDataBleInterfaceParam *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->company_id[0] = data->company_id[0];
  raw->company_id[1] = data->company_id[1];
  raw->data_type = data->data_type;
  raw->_random_window = (uint8_t)ub_round_i32(data->random_window / 0.01f);
  raw->send_count = data->send_count & 0x07;
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
  data->is_sleeping = raw->is_sleeping != 0;
  data->hardware_enabled_uart = raw->hardware_enabled_uart != 0;
  data->hardware_enabled_iic = raw->hardware_enabled_iic != 0;
  data->hardware_enabled_uwb = raw->hardware_enabled_uwb != 0;
  data->hardware_enabled_ble = raw->hardware_enabled_ble != 0;
  data->firmware_series = raw->firmware_series;
  memcpy(data->firmware_version, raw->firmware_version,
         sizeof(data->firmware_version));
  memcpy(data->uid, raw->uid, sizeof(data->uid));
  data->battery_voltage = raw->_battery_voltage / 40.0f;
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
  raw->is_sleeping = data->is_sleeping ? 1 : 0;
  raw->hardware_enabled_uart = data->hardware_enabled_uart ? 1 : 0;
  raw->hardware_enabled_iic = data->hardware_enabled_iic ? 1 : 0;
  raw->hardware_enabled_uwb = data->hardware_enabled_uwb ? 1 : 0;
  raw->hardware_enabled_ble = data->hardware_enabled_ble ? 1 : 0;
  raw->firmware_series = data->firmware_series;
  memcpy(raw->firmware_version, data->firmware_version,
         sizeof(raw->firmware_version));
  memcpy(raw->uid, data->uid, sizeof(raw->uid));
  raw->_battery_voltage =
      (uint8_t)ub_round_i32(data->battery_voltage * 40.0f);
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

static inline void ub_data_anchor_signal_from_raw(
    const UBRawDataAnchorSignal *raw, UBDataAnchorSignal *data) {
  data->local_time_us = raw->local_time_us;
  data->count = (uint8_t)ub_min_int(raw->count, UB_ANCHOR_SIGNAL_DATA_MAX);
  data->area_id = raw->area_id;
  for (int i = 0; i < data->count; ++i) {
    data->datas[i].addr = raw->datas[i].addr;
    data->datas[i].fp_index = raw->datas[i]._fp_index * 0.1f;
    data->datas[i].fp_to_peak = raw->datas[i].fp_to_peak;
    data->datas[i].mc = raw->datas[i]._mc * 0.01f;
    data->datas[i].rx_rssi = raw->datas[i]._rx_rssi * -0.5f;
    data->datas[i].fp_rssi = raw->datas[i]._fp_rssi * -0.5f;
    data->datas[i].uwb_clock_offset_ppm =
        raw->datas[i]._uwb_clock_offset_ppm * 0.01f;
    data->datas[i].mcu_clock_offset_ppm =
        raw->datas[i]._mcu_clock_offset_ppm * 0.01f;
    data->datas[i].rx_rate = raw->datas[i]._rx_rate / 255.0f;
  }
}
static inline void ub_data_anchor_signal_to_raw(const UBDataAnchorSignal *data,
                                                void *buf, int *size) {
  UBRawDataAnchorSignal *raw = (UBRawDataAnchorSignal *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->local_time_us = data->local_time_us;
  int count = ub_min_int(data->count, UB_ANCHOR_SIGNAL_DATA_MAX);
  raw->count = count & 0x0F;
  raw->area_id = data->area_id & 0x0F;
  for (int i = 0; i < count; ++i) {
    raw->datas[i].addr = data->datas[i].addr;
    raw->datas[i]._fp_index =
        (int16_t)ub_round_i32(data->datas[i].fp_index / 0.1f);
    raw->datas[i].fp_to_peak = data->datas[i].fp_to_peak;
    raw->datas[i]._mc = (uint8_t)ub_round_i32(data->datas[i].mc / 0.01f);
    raw->datas[i]._rx_rssi =
        (uint8_t)ub_round_i32(data->datas[i].rx_rssi / -0.5f);
    raw->datas[i]._fp_rssi =
        (uint8_t)ub_round_i32(data->datas[i].fp_rssi / -0.5f);
    raw->datas[i]._uwb_clock_offset_ppm =
        (int16_t)ub_round_i32(data->datas[i].uwb_clock_offset_ppm / 0.01f);
    raw->datas[i]._mcu_clock_offset_ppm =
        (int16_t)ub_round_i32(data->datas[i].mcu_clock_offset_ppm / 0.01f);
    raw->datas[i]._rx_rate =
        (uint8_t)ub_round_i32(data->datas[i].rx_rate * 255.0f);
  }
  // 变长：只发送有效元素
  *size = (int)offsetof(UBRawDataAnchorSignal, datas) +
          count * (int)sizeof(raw->datas[0]);
}

static inline void ub_data_anchor_ddoas_from_raw(
    const UBRawDataAnchorDdoas *raw, UBDataAnchorDdoas *data) {
  data->local_time_us = raw->local_time_us;
  data->count = (uint8_t)ub_min_int(raw->count, UB_ANCHOR_DDOAS_DATA_MAX);
  for (int i = 0; i < data->count; ++i) {
    data->datas[i].a0 = raw->datas[i].a0;
    data->datas[i].a1 = raw->datas[i].a1;
    data->datas[i].ddoa = raw->datas[i].ddoa_cm * 0.01f;
  }
}
static inline void ub_data_anchor_ddoas_to_raw(const UBDataAnchorDdoas *data,
                                               void *buf, int *size) {
  UBRawDataAnchorDdoas *raw = (UBRawDataAnchorDdoas *)buf;
  memset(raw, 0, sizeof(*raw));
  raw->local_time_us = data->local_time_us;
  int count = ub_min_int(data->count, UB_ANCHOR_DDOAS_DATA_MAX);
  raw->count = (uint8_t)count;
  for (int i = 0; i < count; ++i) {
    raw->datas[i].a0 = data->datas[i].a0;
    raw->datas[i].a1 = data->datas[i].a1;
    raw->datas[i].ddoa_cm = (int16_t)ub_round_i32(data->datas[i].ddoa * 100.0f);
  }
  // 变长：只发送有效元素
  *size = (int)offsetof(UBRawDataAnchorDdoas, datas) +
          count * (int)sizeof(raw->datas[0]);
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
