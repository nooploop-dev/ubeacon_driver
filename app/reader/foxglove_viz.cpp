#include "foxglove_viz.hpp"

#include <chrono>
#include <cstddef>
#include <map>
#include <optional>
#include <string>

#include <foxglove/channel.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>

// windows下spdlog带入的头文件会导致foxglove编译报错，因此后置
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

namespace {

namespace fgm = foxglove::messages;

std::optional<foxglove::WebSocketServer> s_server;
std::optional<foxglove::McapWriter> s_writer;
std::optional<fgm::FrameTransformChannel> s_tf_ch;
std::optional<fgm::PoseInFrameChannel> s_pose_ch;
std::optional<fgm::SceneUpdateChannel> s_scene_ch;
std::optional<foxglove::RawChannel> s_location_result_ch;
std::optional<foxglove::RawChannel> s_anchor_signal_ch;
std::optional<foxglove::RawChannel> s_heartbeat_ch;
std::string s_frame_id;

// 信标坐标由 UB_MSG_ANCHOR_POS 陆续上报，缓存下来一并画进 3D 场景
std::map<ub_addr_t, std::array<float, 3>> s_anchors;

// Foxglove 3D 面板需要一个根坐标系，定位结果所在的 frame 挂在它下面
constexpr char ROOT_FRAME[] = "world";

fgm::Timestamp now_ts() {
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
  fgm::Timestamp ts;
  ts.sec = uint32_t(ns / 1000000000);
  ts.nsec = uint32_t(ns % 1000000000);
  return ts;
}

// 供 Foxglove 的 Plot 面板绘制曲线用的 JSON schema
const std::string LOCATION_RESULT_SCHEMA = R"({
  "type": "object",
  "properties": {
    "local_time_us": {"type": "number"},
    "pos_x": {"type": "number"}, "pos_y": {"type": "number"}, "pos_z": {"type": "number"},
    "vel_x": {"type": "number"}, "vel_y": {"type": "number"}, "vel_z": {"type": "number"},
    "pos_noise_x": {"type": "number"}, "pos_noise_y": {"type": "number"}, "pos_noise_z": {"type": "number"},
    "vel_noise_x": {"type": "number"}, "vel_noise_y": {"type": "number"}, "vel_noise_z": {"type": "number"},
    "map_id": {"type": "number"},
    "error_code": {"type": "number"},
    "area_id": {"type": "number"},
    "anchor_count": {"type": "number"}
  }
})";

const std::string ANCHOR_SIGNAL_SCHEMA = R"({
  "type": "object",
  "properties": {
    "local_time_us": {"type": "number"},
    "area_id": {"type": "number"},
    "count": {"type": "number"},
    "datas": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "addr": {"type": "number"},
          "rx_rssi": {"type": "number"},
          "fp_rssi": {"type": "number"},
          "rx_rate": {"type": "number"}
        }
      }
    }
  }
})";

const std::string HEARTBEAT_SCHEMA = R"({
  "type": "object",
  "properties": {
    "battery_percent": {"type": "number"},
    "battery_voltage": {"type": "number"},
    "battery_charging": {"type": "boolean"},
    "restart_cnt": {"type": "number"},
    "is_sleeping": {"type": "boolean"}
  }
})";

foxglove::Schema json_schema(const std::string &name, const std::string &text) {
  foxglove::Schema schema;
  schema.name = name;
  schema.encoding = "jsonschema";
  schema.data = reinterpret_cast<const std::byte *>(text.data());
  schema.data_len = text.size();
  return schema;
}

void log_json(std::optional<foxglove::RawChannel> &ch, const std::string &json) {
  if (!ch) {
    return;
  }
  ch->log(reinterpret_cast<const std::byte *>(json.data()), json.size());
}

// 创建通道，失败时打日志并返回空。通道类型由 FoxgloveResult<T> 推导
template <typename T>
std::optional<T> take(foxglove::FoxgloveResult<T> &&result, const char *topic) {
  if (!result.has_value()) {
    spdlog::warn("Foxglove: failed to create channel '{}': {}", topic,
                 foxglove::strerror(result.error()));
    return std::nullopt;
  }
  return std::optional<T>(std::move(result.value()));
}

// 通道只创建一次，WebSocket 服务与 MCAP 录制都是它的下游(sink)，各自可独立启停
void ensure_channels() {
  static bool created = false;
  if (created) {
    return;
  }
  created = true;

  s_tf_ch = take(fgm::FrameTransformChannel::create("/tf"), "/tf");
  s_pose_ch = take(fgm::PoseInFrameChannel::create("/tag_pose"), "/tag_pose");
  s_scene_ch = take(fgm::SceneUpdateChannel::create("/scene"), "/scene");
  s_location_result_ch = take(
      foxglove::RawChannel::create(
          "/location_result", "json",
          json_schema("ub.LocationResult", LOCATION_RESULT_SCHEMA)),
      "/location_result");
  s_anchor_signal_ch = take(
      foxglove::RawChannel::create(
          "/anchor_signal", "json",
          json_schema("ub.AnchorSignal", ANCHOR_SIGNAL_SCHEMA)),
      "/anchor_signal");
  s_heartbeat_ch =
      take(foxglove::RawChannel::create(
               "/heartbeat", "json",
               json_schema("ub.Heartbeat", HEARTBEAT_SCHEMA)),
           "/heartbeat");
}

} // namespace

namespace foxglove_viz {

bool init(const std::string &host, uint16_t port, const std::string &frame_id) {
  s_frame_id = frame_id;
  ensure_channels();

  foxglove::WebSocketServerOptions options;
  options.name = "ubeacon_reader";
  options.host = host;
  options.port = port;

  auto server = foxglove::WebSocketServer::create(std::move(options));
  if (!server.has_value()) {
    spdlog::error("Foxglove: failed to start server on {}:{}: {}", host, port,
                  foxglove::strerror(server.error()));
    return false;
  }
  s_server.emplace(std::move(server.value()));
  return true;
}

bool record_start(const std::string &file_path) {
  ensure_channels();

  foxglove::McapWriterOptions options;
  options.path = file_path;

  auto writer = foxglove::McapWriter::create(options);
  if (!writer.has_value()) {
    spdlog::error("Record: failed to create '{}': {}", file_path,
                  foxglove::strerror(writer.error()));
    return false;
  }
  s_writer.emplace(std::move(writer.value()));
  return true;
}

void publish(const UBDataLocationResult &data) {
  const auto ts = now_ts();

  // 根坐标系 -> 定位坐标系。3D 面板需要这个 TF，定位点才有落脚的 frame
  if (s_tf_ch) {
    fgm::FrameTransform tf;
    tf.timestamp = ts;
    tf.parent_frame_id = ROOT_FRAME;
    tf.child_frame_id = s_frame_id;
    tf.translation = fgm::Vector3{0, 0, 0};
    tf.rotation = fgm::Quaternion{0, 0, 0, 1};
    s_tf_ch->log(tf);
  }

  // 标签位置。设备不测姿态，姿态填单位四元数
  if (s_pose_ch) {
    fgm::PoseInFrame pose;
    pose.timestamp = ts;
    pose.frame_id = s_frame_id;
    pose.pose = fgm::Pose{fgm::Vector3{data.pos[0], data.pos[1], data.pos[2]},
                          fgm::Quaternion{0, 0, 0, 1}};
    s_pose_ch->log(pose);
  }

  // 3D 场景：标签的不确定性球(直径取 2σ) + 速度矢量线段 + 已知坐标的信标
  if (s_scene_ch) {
    fgm::SpherePrimitive tag;
    tag.pose = fgm::Pose{fgm::Vector3{data.pos[0], data.pos[1], data.pos[2]},
                         fgm::Quaternion{0, 0, 0, 1}};
    tag.size = fgm::Vector3{2.0 * data.pos_noise[0], 2.0 * data.pos_noise[1],
                            2.0 * data.pos_noise[2]};
    tag.color = fgm::Color{0.8, 0.2, 0.8, 0.3};

    fgm::LinePrimitive vel;
    vel.type = fgm::LinePrimitive::LineType::LINE_STRIP;
    vel.thickness = 2.0;
    vel.scale_invariant = true;
    vel.color = fgm::Color{1.0, 0.9, 0.2, 1.0};
    vel.points = {
        fgm::Point3{data.pos[0], data.pos[1], data.pos[2]},
        fgm::Point3{data.pos[0] + data.vel[0], data.pos[1] + data.vel[1],
                    data.pos[2] + data.vel[2]},
    };

    fgm::SceneEntity entity;
    entity.timestamp = ts;
    entity.frame_id = s_frame_id;
    entity.id = "tag";
    entity.spheres = {tag};
    entity.lines = {vel};

    // 参与本次定位的信标标绿，其余已知信标标灰
    fgm::SceneEntity anchors;
    anchors.timestamp = ts;
    anchors.frame_id = s_frame_id;
    anchors.id = "anchors";
    for (const auto &kv : s_anchors) {
      bool used = false;
      for (int i = 0; i < data.anchor_count; ++i) {
        if (data.anchors[i].addr == kv.first) {
          used = true;
          break;
        }
      }
      fgm::CubePrimitive cube;
      cube.pose = fgm::Pose{fgm::Vector3{kv.second[0], kv.second[1],
                                         kv.second[2]},
                            fgm::Quaternion{0, 0, 0, 1}};
      cube.size = fgm::Vector3{0.15, 0.15, 0.15};
      cube.color = used ? fgm::Color{0.2, 0.9, 0.3, 1.0}
                        : fgm::Color{0.5, 0.5, 0.5, 0.6};
      anchors.cubes.push_back(cube);
    }

    fgm::SceneUpdate update;
    update.entities = {entity, anchors};
    s_scene_ch->log(update);
  }

  log_json(s_location_result_ch,
           fmt::format(R"({{"local_time_us":{},)"
                       R"("pos_x":{},"pos_y":{},"pos_z":{},)"
                       R"("vel_x":{},"vel_y":{},"vel_z":{},)"
                       R"("pos_noise_x":{},"pos_noise_y":{},"pos_noise_z":{},)"
                       R"("vel_noise_x":{},"vel_noise_y":{},"vel_noise_z":{},)"
                       R"("map_id":{},"error_code":{},"area_id":{},)"
                       R"("anchor_count":{}}})",
                       data.local_time_us, data.pos[0], data.pos[1], data.pos[2],
                       data.vel[0], data.vel[1], data.vel[2], data.pos_noise[0],
                       data.pos_noise[1], data.pos_noise[2], data.vel_noise[0],
                       data.vel_noise[1], data.vel_noise[2], data.map_id,
                       data.error_code, data.area_id, data.anchor_count));
}

void publish(const UBDataAnchorPos &data) {
  // 缓存信标坐标，供 3D 场景绘制
  s_anchors[data.src] = {data.pos[0], data.pos[1], data.pos[2]};
}

void publish(const UBDataAnchorSignal &data) {
  std::string datas;
  for (int i = 0; i < data.count; ++i) {
    if (i > 0) {
      datas += ",";
    }
    const auto &s = data.datas[i];
    datas += fmt::format(
        R"({{"addr":{},"rx_rssi":{},"fp_rssi":{},"rx_rate":{}}})", s.addr,
        s.rx_rssi, s.fp_rssi, s.rx_rate);
  }
  log_json(s_anchor_signal_ch,
           fmt::format(
               R"({{"local_time_us":{},"area_id":{},"count":{},"datas":[{}]}})",
               data.local_time_us, data.area_id, data.count, datas));
}

void publish(const UBDataHeartbeat &data) {
  log_json(s_heartbeat_ch,
           fmt::format(R"({{"battery_percent":{},"battery_voltage":{},)"
                       R"("battery_charging":{},"restart_cnt":{},)"
                       R"("is_sleeping":{}}})",
                       data.battery_percent, data.battery_voltage,
                       data.battery_charging, data.restart_cnt,
                       data.is_sleeping));
}

void shutdown() {
  if (s_server) {
    s_server->stop();
    s_server.reset();
  }
  if (s_writer) {
    // 不 close 的话 MCAP 缺少索引/统计信息，Foxglove 无法正常打开
    const auto err = s_writer->close();
    if (err != foxglove::FoxgloveError::Ok) {
      spdlog::error("Record: failed to close file: {}",
                    foxglove::strerror(err));
    }
    s_writer.reset();
  }
}

} // namespace foxglove_viz
