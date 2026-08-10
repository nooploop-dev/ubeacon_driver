#pragma once

#include "ubeacon_driver_data.h"

#include <cstdint>
#include <string>

// 把解析出的数据推送到 Foxglove WebSocket 服务，与 spdlog 日志打印并行，互不影响。
// 实现有两份，由 CMake 的 UB_BUILD_FOXGLOVE 在链接期二选一(故调用方无需条件编译)：
//   ON  -> foxglove_viz.cpp(真实现)
//   OFF -> foxglove_viz_stub.cpp(空实现)
namespace foxglove_viz {

// 启动 WebSocket 服务，Foxglove 中通过 ws://<host>:<port> 连接。
// 返回 false 表示未启动(未启用 Foxglove，或启动失败)
bool init(const std::string &host, uint16_t port, const std::string &frame_id);

// 开启 MCAP 录制，publish 的数据在推送 WebSocket 的同时写入 file_path。
// 与 WebSocket 服务相互独立，任一方未启动都不影响另一方。
// 返回 false 表示未开启(未启用 Foxglove，或文件创建失败)
bool record_start(const std::string &file_path);

void publish(const UBDataLocationResult &data);
void publish(const UBDataAnchorPos &data);
void publish(const UBDataAnchorSignal &data);
void publish(const UBDataHeartbeat &data);

// 停止 WebSocket 服务并结束录制(写入 MCAP 索引并关闭文件)
void shutdown();

} // namespace foxglove_viz
