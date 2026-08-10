#include "reader_data_handle.hpp"
#include "foxglove_viz.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <spdlog/fmt/ranges.h>
#include <spdlog/spdlog.h>
#include <string>

namespace {

// 上行帧的设备类型名；非上行 frame_id 返回 NULL
const char *frame_id_name(ub_frame_id_t frame_id) {
  switch (frame_id) {
  case UB_FRAME_ID_GATEWAY_UP:
    return "GATEWAY";
  case UB_FRAME_ID_TAG_UP:
    return "TAG";
  case UB_FRAME_ID_ANCHOR_UP:
    return "ANCHOR";
  default:
    return NULL;
  }
}

std::string data2hex(const void *data, int size) {
  auto p_data = (const uint8_t *)data;
  return fmt::format("{:02X}", fmt::join(p_data, p_data + size, ""));
}

template <typename T, std::size_t N> std::string vec2str(const T (&value)[N]) {
  return fmt::format("[{}]", fmt::join(value, ","));
}

template <typename T>
const T *get_data(const void *msg_payload, int msg_payload_size) {
  assert(msg_payload_size == int(sizeof(T)));
  return static_cast<const T *>(msg_payload);
}

// 以下消息携带变长数组，逐元素展开
std::string anchors2str(const UBDataLocationResult *data) {
  std::string s;
  for (int i = 0; i < data->anchor_count; i++) {
    if (i > 0) {
      s += ",";
    }
    s += fmt::format("({:#06x},{:.1f},{:.2f})", data->anchors[i].addr,
                     data->anchors[i].rx_rssi, data->anchors[i].rx_rate);
  }
  return "[" + s + "]";
}

std::string signals2str(const UBDataAnchorSignal *data) {
  std::string s;
  for (int i = 0; i < data->count; i++) {
    if (i > 0) {
      s += ",";
    }
    const auto &d = data->datas[i];
    s += fmt::format("({:#06x},fp_index={:.1f},fp_to_peak={},mc={:.2f},"
                     "rx_rssi={:.1f},fp_rssi={:.1f},uwb_clk={:.2f},"
                     "mcu_clk={:.2f},rx_rate={:.2f})",
                     d.addr, d.fp_index, d.fp_to_peak, d.mc, d.rx_rssi,
                     d.fp_rssi, d.uwb_clock_offset_ppm, d.mcu_clock_offset_ppm,
                     d.rx_rate);
  }
  return "[" + s + "]";
}

std::string ddoas2str(const UBDataAnchorDdoas *data) {
  std::string s;
  for (int i = 0; i < data->count; i++) {
    if (i > 0) {
      s += ",";
    }
    s += fmt::format("({:#06x},{:#06x},{:.2f})", data->datas[i].a0,
                     data->datas[i].a1, data->datas[i].ddoa);
  }
  return "[" + s + "]";
}

std::string s_header;

} // namespace

namespace data_handle {

bool on_frame_begin(void *arg, const uint8_t *uid, ub_frame_id_t frame_id) {
  (void)arg;
  // 上行帧按设备类型区分，非上行帧(如线路上回环的下行帧)整帧丢弃
  const char *dev = frame_id_name(frame_id);
  if (dev == NULL) {
    spdlog::debug("skip frame: frame_id={}", frame_id);
    return false;
  }
  s_header = fmt::format("{},{}", dev, data2hex(uid, UB_UID_SIZE));
  return true;
}
void on_frame_msg(void *arg, ub_msg_id_t msg_id, const void *msg_payload,
                  int msg_payload_size) {
  (void)arg;
  const auto &header = s_header;
  // 空消息(只有id没有payload)。上行内建消息都带payload，这里只会是扩展消息
  if (msg_payload == nullptr || msg_payload_size == 0) {
    spdlog::info("{},MSG_EMPTY({}): {{}}", header, msg_id);
    return;
  }
  switch (msg_id) {
  case UB_MSG_RESTART: {
    auto data = get_data<UBDataRestart>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_RESTART: {{delay={},only_restart_when_need={}}}",
                 header, data->delay, data->only_restart_when_need);
    break;
  }
  case UB_MSG_FIND: {
    auto data = get_data<UBDataFind>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_FIND: {{duration={}}}", header, data->duration);
    break;
  }
  case UB_MSG_PARAM: {
    auto data = get_data<UBDataParam>(msg_payload, msg_payload_size);
    spdlog::info(
        "{},MSG_PARAM: {{expect_z={:.2f},z_noise={:.2f},smooth_window={},"
        "max_acceleration={},output={{tag_pos={},anchor_packet={},"
        "anchor_pos={},anchor_link_data={},anchor_signal={},anchor_ddoa={},"
        "tag_pos_even_error={},anchor_link_status={}}},sniff_duty_cycle={},"
        "update_interval_max={},reset_interval={}}}",
        header, data->expect_z, data->z_noise, data->smooth_window,
        vec2str(data->max_acceleration), data->output.tag_pos,
        data->output.anchor_packet, data->output.anchor_pos,
        data->output.anchor_link_data, data->output.anchor_signal,
        data->output.anchor_ddoa, data->output.tag_pos_even_error,
        data->output.anchor_link_status, data->sniff_duty_cycle,
        data->update_interval_max, data->reset_interval);
    break;
  }
  case UB_MSG_RUN_TIME_PARAM: {
    auto data = get_data<UBDataRunTimeParam>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_RUN_TIME_PARAM: {{sniff_duty_cycle={}}}", header,
                 data->sniff_duty_cycle);
    break;
  }
  case UB_MSG_INTERFACE_PARAM: {
    auto data = get_data<UBDataInterfaceParam>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_INTERFACE_PARAM: {{uart={},iic={},uwb={},ble={}}}",
                 header, data->uart, data->iic, data->uwb, data->ble);
    break;
  }
  case UB_MSG_UART_INTERFACE_PARAM: {
    auto data =
        get_data<UBDataUartInterfaceParam>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_UART_INTERFACE_PARAM: {{baudrate={}}}", header,
                 data->baudrate);
    break;
  }
  case UB_MSG_IIC_INTERFACE_PARAM: {
    auto data =
        get_data<UBDataIicInterfaceParam>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_IIC_INTERFACE_PARAM: {{addr={:#04x}}}", header,
                 data->addr);
    break;
  }
  case UB_MSG_UWB_INTERFACE_PARAM: {
    auto data =
        get_data<UBDataUwbInterfaceParam>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_UWB_INTERFACE_PARAM: "
                 "{{random_window={:.2f},max_rx_interval={:.2f}}}",
                 header, data->random_window, data->max_rx_interval);
    break;
  }
  case UB_MSG_BLE_INTERFACE_PARAM: {
    auto data =
        get_data<UBDataBleInterfaceParam>(msg_payload, msg_payload_size);
    spdlog::info(
        "{},MSG_BLE_INTERFACE_PARAM: {{company_id={},data_type={:#04x},"
        "random_window={:.2f},send_count={}}}",
        header, vec2str(data->company_id), data->data_type, data->random_window,
        data->send_count);
    break;
  }
  case UB_MSG_LOCATION_RESULT: {
    auto data = get_data<UBDataLocationResult>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_LOCATION_RESULT: {{local_time_us={},pos={},vel={},"
                 "pos_noise={},vel_noise={},map_id={},error_code={},area_id={},"
                 "anchors={}}}",
                 header, data->local_time_us, vec2str(data->pos),
                 vec2str(data->vel), vec2str(data->pos_noise),
                 vec2str(data->vel_noise), data->map_id, data->error_code,
                 data->area_id, anchors2str(data));
    foxglove_viz::publish(*data);
    break;
  }
  case UB_MSG_HEARTBEAT: {
    auto data = get_data<UBDataHeartbeat>(msg_payload, msg_payload_size);
    spdlog::info(
        "{},MSG_HEARTBEAT: {{battery={{percent={},charging={},voltage={:.2f}}},"
        "need_restart={},reset_info_dirty={},assert_info_dirty={},"
        "restart_cnt={},is_sleeping={},hardware_enabled={{uart={},iic={},"
        "uwb={},ble={}}},firmware={{series={},version={}}},uid={}}}",
        header, data->battery_percent, data->battery_charging,
        data->battery_voltage, data->need_restart, data->reset_info_dirty,
        data->assert_info_dirty, data->restart_cnt, data->is_sleeping,
        data->hardware_enabled_uart, data->hardware_enabled_iic,
        data->hardware_enabled_uwb, data->hardware_enabled_ble,
        data->firmware_series, vec2str(data->firmware_version),
        data2hex(data->uid, UB_UID_SIZE));
    foxglove_viz::publish(*data);
    break;
  }
  case UB_MSG_USER_DATA: {
    auto data = get_data<UBDataUserData>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_USER_DATA: {{payload={}}}", header,
                 data2hex(data->payload, data->payload_size));
    break;
  }
  case UB_MSG_ANCHOR_SIGNAL: {
    auto data = get_data<UBDataAnchorSignal>(msg_payload, msg_payload_size);
    spdlog::info(
        "{},MSG_ANCHOR_SIGNAL: {{local_time_us={},area_id={},datas={}}}", header,
        data->local_time_us, data->area_id, signals2str(data));
    foxglove_viz::publish(*data);
    break;
  }
  case UB_MSG_ANCHOR_DDOAS: {
    auto data = get_data<UBDataAnchorDdoas>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_ANCHOR_DDOAS: {{local_time_us={},datas={}}}", header,
                 data->local_time_us, ddoas2str(data));
    break;
  }
  case UB_MSG_ANCHOR_POS: {
    auto data = get_data<UBDataAnchorPos>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_ANCHOR_POS: {{local_time_us={},src={:#06x},pos={},"
                 "is_local_pos={},map_id={},relative_map_z={:.2f}}}",
                 header, data->local_time_us, data->src, vec2str(data->pos),
                 data->is_local_pos, data->map_id, data->relative_map_z);
    foxglove_viz::publish(*data);
    break;
  }
  case UB_MSG_GLOBAL_TIME_STATUS: {
    auto data = get_data<UBDataGlobalTimeStatus>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_GLOBAL_TIME_STATUS: "
                 "{{run={},timeout={},ttl={},time_us={},src={:#06x}}}",
                 header, data->run, data->timeout, data->ttl, data->time_us,
                 data->src);
    break;
  }
  case UB_MSG_Z_MEASUREMENT: {
    auto data = get_data<UBDataZMeasurement>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_Z_MEASUREMENT: {{z={:.2f},z_std={:.2f},timeout={}}}",
                 header, data->z, data->z_std, data->timeout);
    break;
  }
  case UB_MSG_STATE_CONTROL: {
    auto data = get_data<UBDataStateControl>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_STATE_CONTROL: {{sleep={}}}", header, data->sleep);
    break;
  }
  case UB_MSG_MAP_MEASUREMENT: {
    auto data = get_data<UBDataMapMeasurement>(msg_payload, msg_payload_size);
    spdlog::info("{},MSG_MAP_MEASUREMENT: {{map_id={},timeout={}}}", header,
                 data->map_id, data->timeout);
    break;
  }
  default: {
    spdlog::info("{},MSG_UNKNOWN({}): {{raw_payload={}}}", header, msg_id,
                 data2hex(msg_payload, msg_payload_size));
    break;
  }
  }
}
void on_frame_end(void *arg) { (void)arg; }

} // namespace data_handle
