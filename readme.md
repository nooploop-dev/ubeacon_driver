# UBEACON DEVICE DRIVER

简体中文 | [English](./readme.en.md)

本仓库为 Nooploop [uBeacon 定位系统](https://support.nooploop.com/cn/ubeacon/) 标签(Tag)的驱动代码。它提供：

- **纯 C 驱动**（`include/` + `src/`）：无任何第三方依赖，可直接集成进单片机/嵌入式工程，负责设备通信协议的「组包」与「解析」。
- **ROS1 / ROS2 驱动包**（`app/` + `msg/` + `launch/`）：在纯 C 驱动之上封装串口收发与话题转换，clone 到 ROS 工作空间即可使用。

## 目录结构

```text
ubeacon_driver/
├── include/                        # 对外公共头
│   ├── ubeacon_driver_for_user.h   #   用户端接口(集成时通常只需这一个)
│   ├── ubeacon_driver_data.h       #   消息ID与消息结构体定义(被上面的头包含)
│   └── ubeacon_driver_common.h     #   公共基础类型与协议常量
├── src/                            # 纯C驱动实现(需一起加入编译)
│   ├── ubeacon_checksum.h          #   累加和校验
│   ├── ubeacon_frame.c/.h          #   帧组装/拆解
│   ├── ubeacon_driver.c            #   组包/解析主逻辑
│   ├── ubeacon_driver_data_raw.h   #   协议层结构体与转换(内部)
│   └── ubeacon_msg.h               #   消息层(内部)
├── test/                           # 接口用法的完整示例(强烈建议参考)
├── app/                            # 上位机应用与集成示例
│   ├── reader/                     #   从串口接收并解析(含可选 Foxglove 可视化)
│   ├── writer/                     #   组包并通过串口向设备发送
│   ├── ros1_converter/             #   ROS1 话题转换节点
│   └── ros2_converter/             #   ROS2 话题转换节点
├── msg/                            # ROS 消息定义
├── launch/                         # ROS launch 文件
├── rviz/                           # rviz 显示配置(ROS1/ROS2 各一份)
├── package.xml                     # ROS 包描述
└── CMakeLists.txt
```

## 纯 C 驱动集成（单片机 / 嵌入式）

纯 C 驱动只做两件事：把要发给设备的消息**组装成数据帧**，以及把从设备收到的字节流**解析成消息**。数据的实际收发由用户自行实现。

### 加入工程

- 源文件：`src/ubeacon_frame.c`、`src/ubeacon_driver.c`
- 头文件目录：`include/`

业务代码中只需 `#include "ubeacon_driver_for_user.h"`。驱动遵循 C11，无动态内存分配、无第三方依赖，适合资源受限的 MCU。

> 纯 C 工程不需要 `app/`。若想了解如何把驱动接口接入实际收发流程，可参考 [`app/reader/`](app/reader/)（解析）和 [`app/writer/`](app/writer/)（组包）。示例用 C++ 实现串口与日志，但调用的驱动接口与纯 C 工程完全相同。

### 用户接口与协议细节分离

驱动为每个消息维护**两族结构体**：公开的 `UBData*`（真实物理单位、`float`、`bool`、正常对齐）与内部的 `UBRawData*`（`pack(1)`、位域、定标整数、保留字段）。两者由一对纯函数桥接，**所有定标魔数集中在内部的转换器里**，用户接触不到字节、位域、校验和与字节序。

公私边界由目录强制：**应用只包含 `include/` 下的头文件，永不包含 `src/` 中的任何东西。**

线协议：

```text
帧：          sof(0xAA) | payload_size(2B 小端) | payload | checksum(1B 累加和)
上行 payload： uid(6B) | frame_id | msg msg msg ...
下行 payload：           frame_id | msg msg msg ...
消息：        id(1B) | payload_size(7bit) | payload
```

`frame_id`（`ub_frame_id_e`）同时标明方向与设备类型：下行统一为 `UB_FRAME_ID_DOWN`，上行按发送方分为 `UB_FRAME_ID_GATEWAY_UP` / `UB_FRAME_ID_TAG_UP` / `UB_FRAME_ID_ANCHOR_UP`。

### 接口概览

接口定义在 [include/ubeacon_driver_for_user.h](include/ubeacon_driver_for_user.h)，消息 ID 与消息结构体定义在 [include/ubeacon_driver_data.h](include/ubeacon_driver_data.h)（前者已包含后者，业务代码只需 include 前者）。通信分「设备 → 用户」和「用户 → 设备」两个方向：

- **组包**：`ub_prepare_msg_to_dev()` 将一个消息（`UBData*` 结构体）封装为一帧，返回帧长度，随后把该帧通过串口发给设备。`READ_*` 等无负载的读请求，`data` 传 `NULL` 即可。
  - 如需在一帧里打包多条消息，使用分步接口：`ub_prepare_msg_to_dev_begin()` → `ub_prepare_msg_to_dev_try_append()`（可多次）→ `ub_prepare_msg_to_dev_end()`。
- **解析**：用 `UBParserFromDev` + `ub_parser_from_dev_init()` 注册回调，串口每收到一段数据就喂给 `ub_parser_from_dev_handle_data()`，内部自动完成拆帧与解析。每帧开始时回调 `on_frame_begin`（提供设备 uid 与该帧的 `frame_id`），帧内每解析出一条消息回调一次 `on_frame_msg`（按 `msg_id` 取对应的 `UBData*` 结构体），处理完一帧后回调 `on_frame_end`；不需要的回调可传 `NULL`。三个回调都会把注册时传入的 `arg` 原样传回。
  - `on_frame_begin` 返回 `bool`：返回 `true` 才会继续解析该帧内的消息并在结束时回调 `on_frame_end`；返回 `false` 则整帧丢弃。**驱动自身不按 `frame_id` 过滤**，由用户在此回调中判断该帧是否来自自己所接的设备（`UB_FRAME_ID_TAG_UP` 等）。未注册该回调时任何帧都会被按上行帧解析——由于上下行 payload 头部长度不同，收发线路上可能出现下行帧时务必注册此回调。

> 消息 ID（`UB_MSG_*`）与结构体（`UBData*`）的一一映射、各字段含义与单位，均见 [include/ubeacon_driver_data.h](include/ubeacon_driver_data.h) 的注释。
>
> 若你开发的是**设备端固件**（需要组包发给用户），接口见 [include/ubeacon_driver_for_dev.h](include/ubeacon_driver_for_dev.h)。

### 最小示例

```c
#include "ubeacon_driver_for_user.h"
#include <stdio.h>

// —— 接收：每解析出一条设备上报的消息时被回调 ——
// arg 为 ub_parser_from_dev_init 注册时传入的指针，可用于携带上下文
static void on_frame_msg_from_dev(void *arg, ub_msg_id_t msg_id,
                                  const void *data, int data_size) {
  (void)arg;
  (void)data_size;
  switch (msg_id) {
  case UB_MSG_LOCATION_RESULT: { // 定位结果
    const UBDataLocationResult *r = (const UBDataLocationResult *)data;
    printf("pos=%.3f %.3f %.3f\n", r->pos[0], r->pos[1], r->pos[2]);
    break;
  }
  default:
    break;
  }
}

// —— 每帧开始时被回调，返回 false 可整帧丢弃 ——
// 据此判断该帧是否来自自己所接的设备
static bool on_frame_begin_from_dev(void *arg, const uint8_t *uid,
                                    ub_frame_id_t frame_id) {
  (void)arg;
  (void)uid;
  return frame_id == UB_FRAME_ID_TAG_UP; // 只处理标签上行帧
}

static UBParserFromDev g_parser;

void app_init(void) {
  // 不关心帧头信息时，on_frame_begin/on_frame_end 都可以传 NULL
  ub_parser_from_dev_init(&g_parser, on_frame_begin_from_dev,
                          on_frame_msg_from_dev, NULL, NULL);
}

// 串口中断/轮询收到数据后调用
void app_on_serial_rx(const uint8_t *data, int size) {
  ub_parser_from_dev_handle_data(&g_parser, data, size);
}

// —— 发送：向设备下发一条命令(例如让设备震动/闪灯提示10秒) ——
void app_send_find(void) {
  UBDataFind find = {0};
  find.duration = 10;

  uint8_t frame[UB_FRAME_SIZE_MAX];
  int n = ub_prepare_msg_to_dev(UB_MSG_FIND, &find, frame, sizeof(frame));
  // user_serial_write 为用户自行实现的串口发送函数
  user_serial_write(frame, n);
}
```

### 扩展新消息

在**不修改本驱动**的前提下新增自定义消息：定义宏 `UBEACON_DRIVER_DATA_EXTEND_ENABLED`，并实现两个钩子函数，编解码 switch 的 `default:` 分支会委托给它们，不支持的 `msg_id` 返回 `-1` 即可：

```c
int ub_encode_extend(ub_msg_id_t msg_id, const void *data,
                     void *raw, int raw_size_max);
int ub_decode_extend(ub_msg_id_t msg_id, const void *payload, int payload_size,
                     void *data_buf, int data_buf_size);
```

**版本兼容**由 `ub_msg_copy_payload` 保证：收到的 payload 比结构短则补零、长则截断。新固件在尾部追加字段不会打挂旧驱动，反之亦然。

### 更多用法

[test/test_ubeacon_driver.cpp](test/test_ubeacon_driver.cpp) 覆盖了**所有消息**在两个方向上的组包与解析往返用例，是最完整、最权威的接口用法参考，集成时可直接对照。

## ROS 集成

ROS1 与 ROS2 共用同一份代码，节点名与可执行文件名均为 `ubeacon_driver`。构建时依据环境变量 `ROS_VERSION` 自动选择版本，无需手动指定。

> 前置：已安装 ROS 并完成 `source /opt/ros/<distro>/setup.bash`。

### 编译

把本仓库 clone 到工作空间的 `src/` 下（目录名为 `ubeacon_driver`），然后：

| | ROS1 | ROS2 |
| --- | --- | --- |
| 编译 | `catkin_make` | `colcon build` |
| 环境 | `source devel/setup.bash` | `source install/setup.bash` |

### 运行

把 uBeacon 标签通过串口接入电脑，确认串口号后启动：

```bash
roslaunch ubeacon_driver msg.launch port:=/dev/ttyUSB0     # ROS1
ros2 launch ubeacon_driver msg.py   port:=/dev/ttyUSB0     # ROS2
```

两者通用的参数：

- `port`：串口设备号
- `baudrate`：波特率，默认 `115200`
- `frame_id`：定位结果所在坐标系，写入 `header.frame_id`，默认 `map`

ROS2 也可不用 launch 直接运行节点：

```bash
ros2 run ubeacon_driver ubeacon_driver --ros-args -p port:=/dev/ttyUSB0
```

> 若启动时报串口「Permission denied」，见下文 [串口权限（Linux）](#串口权限linux)。

### 可视化（rviz）

用 `rviz` 这套 launch 一步启动「驱动 + 静态 TF + rviz」，开箱即可看到标签位置：

```bash
roslaunch ubeacon_driver rviz.launch port:=/dev/ttyUSB0    # ROS1
ros2 launch ubeacon_driver rviz.py   port:=/dev/ttyUSB0    # ROS2
```

除上面的参数外，还支持：

- `parent_frame`：地图坐标系的父坐标系，默认 `world`
- 地图在 `parent_frame` 中的位姿：ROS1 用 `map_pose:="x y z yaw pitch roll"`（默认全 0）；ROS2 用 `x`/`y`/`z`/`yaw`/`pitch`/`roll` 分别指定
- `static_tf`：是否发布 `parent_frame` → `frame_id` 的静态 TF，默认 `true`。若该 TF 已由其它节点（如机器人 URDF）发布，置 `false` 避免冲突

> 定位结果发布在地图坐标系 `map` 下，地图与世界的关系属于上层标定的职责，驱动本身不发布 TF。上面的静态 TF 只是为了让裸测时 rviz 有一棵可用的 TF 树。

### 查看数据

```bash
rostopic echo /ubeacon_driver/location_result       # ROS1
ros2 topic echo /ubeacon_driver/location_result     # ROS2
```

### 话题

话题均以节点名为命名空间，即 `/ubeacon_driver/<话题名>`。消息字段定义见 [msg/](msg/) 目录。

**设备 → 用户**（订阅以获取数据）

| 话题 | 消息类型 | 说明 |
| --- | --- | --- |
| `~/location_result` | `LocationResult` | 定位结果（位置/速度/噪声/参与定位的信标） |
| `~/location_result_pose` | `geometry_msgs/PoseStamped` | 同一份定位结果的标准消息，供 rviz 直接显示 |
| `~/anchor_pos` | `AnchorPos` | 标签收到的信标坐标信息 |
| `~/anchor_signal` | `AnchorSignal` | 每次定位对应的所有信标信号数据 |
| `~/anchor_ddoas` | `AnchorDdoas` | 标签到信标间的距离差 |
| `~/global_time_status` | `GlobalTimeStatus` | 全局时间同步状态 |
| `~/heartbeat` | `Heartbeat` | 设备心跳/状态信息 |
| `~/param` | `Param` | ROM 工作参数读取响应/当前参数 |
| `~/run_time_param` | `RunTimeParam` | RAM 运行时参数 |
| `~/interface_param` | `InterfaceParam` | 对外通信方式配置 |
| `~/uart_interface_param` | `UartInterfaceParam` | 串口通信参数 |
| `~/iic_interface_param` | `IicInterfaceParam` | iic 通信参数 |
| `~/uwb_interface_param` | `UwbInterfaceParam` | uwb 通信参数 |
| `~/ble_interface_param` | `BleInterfaceParam` | ble 通信参数 |
| `~/user_data_from_device` | `UserData` | 设备上报的用户自定义数据 |

**用户 → 设备**（发布以下发命令）

| 话题 | 消息类型 | 说明 |
| --- | --- | --- |
| `~/find` | `Find` | 让设备震动/闪灯以便寻找 |
| `~/restart` | `Restart` | 重启设备 |
| `~/state_control` | `StateControl` | 控制标签睡眠/唤醒 |
| `~/z_measurement` | `ZMeasurement` | 输入准确的标签 z 坐标 |
| `~/map_measurement` | `MapMeasurement` | 输入标签所在地图 |
| `~/user_data_to_device` | `UserData` | 下发用户自定义数据 |
| `~/param_read` | `std_msgs/Empty` | 读取 ROM 工作参数 |
| `~/param_write` | `Param` | 写入 ROM 工作参数 |
| `~/run_time_param_read` | `std_msgs/Empty` | 读取 RAM 运行时参数 |
| `~/run_time_param_write` | `RunTimeParam` | 写入 RAM 运行时参数 |
| `~/interface_param_read` | `std_msgs/Empty` | 读取对外通信方式配置 |
| `~/uart_interface_param_read` | `std_msgs/Empty` | 读取串口通信参数 |
| `~/iic_interface_param_read` | `std_msgs/Empty` | 读取 iic 通信参数 |
| `~/uwb_interface_param_read` | `std_msgs/Empty` | 读取 uwb 通信参数 |
| `~/ble_interface_param_read` | `std_msgs/Empty` | 读取 ble 通信参数 |

## Foxglove 可视化（无需 ROS）

`app/reader/` 在原有日志打印之外，可内置一个 Foxglove WebSocket 服务，不依赖 ROS，适合在没有 ROS 环境的机器上直接看数据。**非 ROS 构建下默认开启**：

```bash
cmake -B build && cmake --build build
./build/app/reader/reader --port /dev/ttyUSB0   # 波特率默认 115200，Foxglove 端口默认 8765
./build/app/reader/reader --port /dev/ttyUSB0 --baudrate 115200 --foxglove_port 8765
```

然后在 Foxglove 中选择 **Open connection → Foxglove WebSocket**，地址填 `ws://<设备IP>:8765`。日志打印保持原样，两者同时输出。

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/tag_pose` | `foxglove.PoseInFrame` | 标签位置，3D 面板显示 |
| `/scene` | `foxglove.SceneUpdate` | 不确定性球（直径 2σ）+ 速度矢量 + 已知坐标的信标 |
| `/tf` | `foxglove.FrameTransform` | `world` → `map`，3D 面板的坐标系 |
| `/location_result` | JSON | 位置/速度/噪声/error_code，供 Plot 面板绘制曲线 |
| `/anchor_signal` | JSON | 信标信号强度/收包率，供 Plot 面板绘制曲线 |
| `/heartbeat` | JSON | 电量等状态，供 Plot 面板绘制曲线 |

信标坐标来自 `UB_MSG_ANCHOR_POS`，会被缓存并画进 3D 场景；参与本次定位的信标标绿，其余标灰。

### 数据录制（MCAP）

加 `--record` 即可把上表中的所有话题录制成 MCAP 文件，事后可直接拖进 Foxglove 回放：

```bash
./build/app/reader/reader --port /dev/ttyUSB0 --record
```

文件写入 `logs/ubeacon_<日期时间>.mcap`（与日志同目录，例如 `logs/ubeacon_20260713_182432_123.mcap`，精确到毫秒，多次运行不会互相覆盖）。录制与 WebSocket 服务相互独立，可同时使用。

> 录制依赖 Foxglove SDK，需要 `UB_BUILD_FOXGLOVE=ON`（非 ROS 构建下默认开启）。请用 `Ctrl+C` 正常退出，程序会在退出时写入索引并关闭文件；强杀（`kill -9`）会导致 MCAP 缺少索引而无法打开。

## 串口权限（Linux）

首次访问串口时常见如下报错：

```text
could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0'
```

原因是串口设备属于 `dialout` 组（部分发行版为 `uucp`），而普通用户默认不在该组。用 `ls -l /dev/ttyUSB0` 确认组名后，把当前用户加入该组，**重新登录**生效：

```bash
sudo usermod -a -G dialout $USER   # -a 不可省略，否则会把用户移出其它附加组
```

参考：[Fix Serial Port “Permission Denied” Errors on Linux](https://websistent.com/fix-serial-port-permission-denied-errors-linux/)

## 构建选项（CMake）

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `UB_BUILD_ROS1` | 依 `ROS_VERSION` 自动检测 | 构建 ROS1 驱动包 |
| `UB_BUILD_ROS2` | 依 `ROS_VERSION` 自动检测 | 构建 ROS2 驱动包 |
| `UB_BUILD_READER` | 非 ROS 构建 ON / ROS 构建 OFF | 构建串口接收与解析示例 |
| `UB_BUILD_WRITER` | 非 ROS 构建 ON / ROS 构建 OFF | 构建组包与串口发送示例 |
| `UB_BUILD_FOXGLOVE` | 非 ROS 构建 ON / ROS 构建 OFF | 为 reader 启用 Foxglove 可视化（自动拉取 Foxglove SDK） |
| `UB_BUILD_TEST` | 非 ROS 构建 ON / ROS 构建 OFF | 构建纯 C 驱动单元测试（自动拉取 Catch2） |

未显式指定 ROS 选项时，会依据环境变量 `ROS_VERSION` 自动选择，因此在 ROS 工作空间中直接用 `catkin_make` / `colcon build` 即可。

一旦启用 ROS1 或 ROS2，后四个选项默认全部关闭：ROS 构建只产出 `ubeacon_driver` 节点，不会连带编译上位机示例、单元测试，也不会去拉取 Catch2 / Foxglove SDK。如仍需其中某项，显式打开即可，例如在 ROS 工作空间里编译并运行单元测试：

```bash
catkin_make -DUB_BUILD_TEST=ON                              # ROS1
colcon build --cmake-args -DUB_BUILD_TEST=ON                # ROS2
```

非 ROS 环境下也可直接用 `CMakePresets.json` 中的预设（Ninja，含 windows/linux/macos × x64/arm64 × debug/release）：

```bash
cmake --preset linux-x64-release
cmake --build build/linux-x64-release
ctest --test-dir build/linux-x64-release --output-on-failure
```
