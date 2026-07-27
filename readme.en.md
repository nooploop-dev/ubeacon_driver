# UBEACON DEVICE DRIVER

[简体中文](./readme.md) | English

This repository contains the driver code for the Tag of the Nooploop [uBeacon positioning system](https://support.nooploop.com/en/ubeacon/). It provides:

- **Pure C driver** (`include/` + `src/`): no third-party dependencies, ready to be integrated into MCU/embedded projects. It handles the "encoding" and "parsing" of the device communication protocol.
- **ROS1 / ROS2 driver packages** (`app/` + `msg/` + `launch/`): a layer on top of the pure C driver that wraps serial I/O and topic conversion. Just clone it into a ROS workspace to use it.

## Directory Structure

```text
ubeacon_driver/
├── include/                        # Public headers
│   ├── ubeacon_driver_for_user.h   #   User-side API (usually the only one you need)
│   ├── ubeacon_driver_data.h       #   Message IDs and structs (included by the header above)
│   └── ubeacon_driver_common.h     #   Common base types and protocol constants
├── src/                            # Pure C driver implementation (compile these too)
│   ├── ubeacon_checksum.h          #   Additive checksum
│   ├── ubeacon_frame.c/.h          #   Frame assembly/disassembly
│   ├── ubeacon_driver.c            #   Encoding/parsing main logic
│   ├── ubeacon_driver_data_raw.h   #   Protocol-layer structs and conversions (internal)
│   └── ubeacon_msg.h               #   Message layer (internal)
├── test/                           # Complete API usage examples (highly recommended)
├── app/                            # Host applications and integration examples
│   ├── reader/                     #   Receive and parse over serial (optional Foxglove viz)
│   ├── writer/                     #   Encode and send to the device over serial
│   ├── ros1_converter/             #   ROS1 topic conversion node
│   └── ros2_converter/             #   ROS2 topic conversion node
├── msg/                            # ROS message definitions
├── launch/                         # ROS launch files
├── rviz/                           # rviz display configs (one each for ROS1/ROS2)
├── package.xml                     # ROS package manifest
└── CMakeLists.txt
```

## Pure C Driver Integration (MCU / Embedded)

The pure C driver does just two things: **encode** the messages you want to send to the device into data frames, and **parse** the byte stream received from the device into messages. The actual I/O is left to you.

### Add to Your Project

- Source files: `src/ubeacon_frame.c`, `src/ubeacon_driver.c`
- Include directory: `include/`

Your application code only needs `#include "ubeacon_driver_for_user.h"`. The driver targets C11, performs no dynamic allocation and has no third-party dependencies, so it fits resource-constrained MCUs.

> A pure C project does not need `app/`. To see how the driver API plugs into a real I/O loop, look at [`app/reader/`](app/reader/) (parsing) and [`app/writer/`](app/writer/) (encoding). Those examples use C++ for serial and logging, but the driver API they call is exactly the same as in a pure C project.

### User API Separated From Protocol Details

The driver keeps **two struct families** per message: the public `UBData*` (real physical units, `float`, `bool`, natural alignment) and the internal `UBRawData*` (`pack(1)`, bitfields, scaled integers, reserved fields). The two are bridged by a pair of pure functions, so **every scaling constant lives inside those internal converters** and users never touch a wire byte, a bitfield, a checksum, or an endianness concern.

The public/private boundary is enforced by directory: **applications include only headers under `include/`, never anything from `src/`.**

Wire protocol:

```text
Frame:            sof(0xAA) | payload_size(2B LE) | payload | checksum(1B additive)
Uplink payload:   uid(6B) | frame_id | msg msg msg ...
Downlink payload:           frame_id | msg msg msg ...
Message:          id(1B) | payload_size(7bit) | payload
```

`frame_id` (`ub_frame_id_e`) encodes both direction and device type: downlink is always `UB_FRAME_ID_DOWN`, while uplink is `UB_FRAME_ID_GATEWAY_UP` / `UB_FRAME_ID_TAG_UP` / `UB_FRAME_ID_ANCHOR_UP` depending on the sender.

### API Overview

The API is defined in [include/ubeacon_driver_for_user.h](include/ubeacon_driver_for_user.h); message IDs and structs live in [include/ubeacon_driver_data.h](include/ubeacon_driver_data.h) (the former already includes the latter, so your code only needs the former). Communication has two directions, "device → user" and "user → device":

- **Encoding**: `ub_prepare_msg_to_dev()` wraps one message (a `UBData*` struct) into a frame and returns the frame length; send that frame to the device over serial. For payload-less read requests such as `READ_*`, pass `NULL` as `data`.
  - To pack several messages into one frame, use the step-by-step API: `ub_prepare_msg_to_dev_begin()` → `ub_prepare_msg_to_dev_try_append()` (repeatable) → `ub_prepare_msg_to_dev_end()`.
- **Parsing**: register callbacks with `UBParserFromDev` + `ub_parser_from_dev_init()`, then feed every chunk of serial data to `ub_parser_from_dev_handle_data()`; framing and parsing happen internally. `on_frame_begin` fires at the start of each frame (giving the device uid and the frame's `frame_id`), `on_frame_msg` fires once per message in the frame (cast to the `UBData*` struct selected by `msg_id`), and `on_frame_end` fires after the frame is done. Pass `NULL` for callbacks you do not need. All three receive the `arg` you registered, unchanged.
  - `on_frame_begin` returns `bool`: only `true` lets the driver go on to parse the messages in that frame and fire `on_frame_end`; `false` drops the whole frame. **The driver itself never filters on `frame_id`** — it is up to you to decide in this callback whether the frame came from the device you are attached to (`UB_FRAME_ID_TAG_UP` and friends). If the callback is not registered, every frame is parsed as an uplink frame; since the uplink and downlink payload headers differ in length, always register it when downlink frames may appear on the same line.

**Messages are direction-scoped**: every `UB_MSG_*` in [include/ubeacon_driver_data.h](include/ubeacon_driver_data.h) is annotated with its direction — `v` for downlink (host → device), `^` for uplink (device → host), `v^` for both. The one-to-one mapping between a `msg_id` and a struct **only holds within a single direction**, and the driver enforces it:

- Encoding a `msg_id` that does not exist in that direction returns `-1` (e.g. passing the uplink-only `UB_MSG_LOCATION_RESULT` to `ub_prepare_msg_to_dev`)
- Parsing a `msg_id` that does not exist in that direction skips just that message; the rest of the frame is still processed

> The one-to-one mapping between message IDs (`UB_MSG_*`) and structs (`UBData*`), plus the meaning and unit of every field, is documented in the comments of [include/ubeacon_driver_data.h](include/ubeacon_driver_data.h).
>
> If you are writing **device firmware** (encoding messages to send to the user), see [include/ubeacon_driver_for_dev.h](include/ubeacon_driver_for_dev.h).

### Minimal Example

```c
#include "ubeacon_driver_for_user.h"
#include <stdio.h>

// -- Receiving: called for each message parsed from the device --
// arg is whatever pointer you passed to ub_parser_from_dev_init; use it for context
static void on_frame_msg_from_dev(void *arg, ub_msg_id_t msg_id,
                                  const void *data, int data_size) {
  (void)arg;
  (void)data_size;
  switch (msg_id) {
  case UB_MSG_LOCATION_RESULT: { // positioning result
    const UBDataLocationResult *r = (const UBDataLocationResult *)data;
    printf("pos=%.3f %.3f %.3f\n", r->pos[0], r->pos[1], r->pos[2]);
    break;
  }
  default:
    break;
  }
}

// -- Called at the start of each frame; return false to drop the whole frame --
// Use it to tell whether the frame came from the device you are attached to
static bool on_frame_begin_from_dev(void *arg, const uint8_t *uid,
                                    ub_frame_id_t frame_id) {
  (void)arg;
  (void)uid;
  return frame_id == UB_FRAME_ID_TAG_UP; // handle tag uplink frames only
}

static UBParserFromDev g_parser;

void app_init(void) {
  // Pass NULL for on_frame_begin/on_frame_end if you do not need frame header info
  ub_parser_from_dev_init(&g_parser, on_frame_begin_from_dev,
                          on_frame_msg_from_dev, NULL, NULL);
}

// Call this from your serial ISR/polling loop
void app_on_serial_rx(const uint8_t *data, int size) {
  ub_parser_from_dev_handle_data(&g_parser, data, size);
}

// -- Sending: issue a command to the device (e.g. vibrate/blink for 10 s) --
void app_send_find(void) {
  UBDataFind find = {0};
  find.duration = 10;

  uint8_t frame[UB_FRAME_SIZE_MAX];
  int n = ub_prepare_msg_to_dev(UB_MSG_FIND, &find, frame, sizeof(frame));
  // user_serial_write is your own serial transmit function
  user_serial_write(frame, n);
}
```

### Adding New Messages

To add custom messages **without modifying this driver**: write your own encode/decode functions and inject them as function pointers. The hooks are consulted only when the built-in message table misses, so they **cannot override built-in messages**. Injecting nothing means no extension messages are supported (the default).

Both hook signatures live in [include/ubeacon_driver_common.h](include/ubeacon_driver_common.h):

```c
// Convert data into wire format in raw, return the byte count; -1 if unsupported
typedef int (*ub_encode_extend_f)(ub_msg_id_t msg_id, const void *data,
                                  void *raw, int raw_size_max);
// Convert payload into a UBData* in data_buf, return the byte count; -1 if unsupported
typedef int (*ub_decode_extend_f)(ub_msg_id_t msg_id, const void *payload,
                                  int payload_size, void *data_buf,
                                  int data_buf_size);
```

The injection points are split across the two headers by which side you are on, so each side only deals with the two directions it actually uses:

```c
// ubeacon_driver_for_user.h -- host side: sends downlink, receives uplink
void ub_set_encode_user_to_dev_extend(ub_encode_extend_f encode);
void ub_set_decode_dev_to_user_extend(ub_decode_extend_f decode);

// ubeacon_driver_for_dev.h -- device side: sends uplink, receives downlink
void ub_set_encode_dev_to_user_extend(ub_encode_extend_f encode);
void ub_set_decode_user_to_dev_extend(ub_decode_extend_f decode);
```

They are split per direction because the one-to-one mapping between a `msg_id` and a struct **only holds within a single direction**. For a custom message used in only one direction, simply do not inject the other. The driver's built-in messages are likewise split into four tables (`ub_encode_dev_to_user` / `ub_encode_user_to_dev` / `ub_decode_dev_to_user` / `ub_decode_user_to_dev`).

Injection is process-global, normally done once at startup; pass `NULL` to undo it:

```c
static int my_encode(ub_msg_id_t msg_id, const void *data,
                     void *raw, int raw_size_max) {
  if (msg_id != MY_MSG_ID || raw_size_max < 2) return -1;
  /* ...write into raw... */
  return 2;
}

ub_set_encode_user_to_dev_extend(my_encode);
```

**Version compatibility** is guaranteed by `ub_msg_copy_payload`: a received payload shorter than the struct is zero-padded, a longer one is truncated. New firmware appending fields at the tail will not break an old driver, and vice versa.

### More Usage

[test/test_ubeacon_driver.cpp](test/test_ubeacon_driver.cpp) covers encode/parse round trips for **every message** in each of its valid directions (and checks that a message used in the wrong direction is rejected). It is the most complete and authoritative API usage reference — keep it open while integrating.

## ROS Integration

ROS1 and ROS2 share the same code; both the node name and the executable name are `ubeacon_driver`. The version is selected automatically at build time from the `ROS_VERSION` environment variable, so you never have to specify it manually.

> Prerequisite: ROS is installed and you have run `source /opt/ros/<distro>/setup.bash`.

### Build

Clone this repository into your workspace's `src/` directory (as `ubeacon_driver`), then:

| | ROS1 | ROS2 |
| --- | --- | --- |
| Build | `catkin_make` | `colcon build` |
| Environment | `source devel/setup.bash` | `source install/setup.bash` |

### Run

Connect the uBeacon tag to your computer over serial, confirm the port, and launch:

```bash
roslaunch ubeacon_driver msg.launch port:=/dev/ttyUSB0     # ROS1
ros2 launch ubeacon_driver msg.py   port:=/dev/ttyUSB0     # ROS2
```

Arguments common to both:

- `port`: serial device path
- `baudrate`: baud rate, default `115200`
- `frame_id`: frame the positioning result is expressed in, written to `header.frame_id`, default `map`

On ROS2 you can also run the node directly without a launch file:

```bash
ros2 run ubeacon_driver ubeacon_driver --ros-args -p port:=/dev/ttyUSB0
```

> If startup fails with a serial "Permission denied", see [Serial Port Permissions (Linux)](#serial-port-permissions-linux) below.

### Visualization (rviz)

The `rviz` launch files start "driver + static TF + rviz" in one step, so you can see the tag position out of the box:

```bash
roslaunch ubeacon_driver rviz.launch port:=/dev/ttyUSB0    # ROS1
ros2 launch ubeacon_driver rviz.py   port:=/dev/ttyUSB0    # ROS2
```

In addition to the arguments above:

- `parent_frame`: parent frame of the map frame, default `world`
- Map pose within `parent_frame`: on ROS1 use `map_pose:="x y z yaw pitch roll"` (all `0` by default); on ROS2 set `x`/`y`/`z`/`yaw`/`pitch`/`roll` individually
- `static_tf`: whether to publish the `parent_frame` → `frame_id` static TF, default `true`. Set to `false` if that TF is already published elsewhere (e.g. by your robot URDF) to avoid conflicts

> The positioning result is published in the map frame `map`. How the map relates to the world belongs to your calibration layer, so the driver itself publishes no TF. The static TF above merely gives rviz a usable TF tree for standalone testing.

### View Data

```bash
rostopic echo /ubeacon_driver/location_result       # ROS1
ros2 topic echo /ubeacon_driver/location_result     # ROS2
```

### Topics

All topics are namespaced under the node name, i.e. `/ubeacon_driver/<topic>`. Message field definitions are in the [msg/](msg/) directory.

**Device → User** (subscribe to receive data)

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/location_result` | `LocationResult` | Positioning result (position/velocity/noise/anchors used) |
| `~/location_result_pose` | `geometry_msgs/PoseStamped` | The same result as a standard message, for direct display in rviz |
| `~/anchor_pos` | `AnchorPos` | Anchor coordinates received by the tag |
| `~/anchor_signal` | `AnchorSignal` | Signal data of all anchors for one positioning cycle |
| `~/anchor_ddoas` | `AnchorDdoas` | Distance differences from the tag to anchor pairs |
| `~/global_time_status` | `GlobalTimeStatus` | Global time synchronization status |
| `~/heartbeat` | `Heartbeat` | Device heartbeat/status info |
| `~/param` | `Param` | ROM parameter read response/current parameters |
| `~/run_time_param` | `RunTimeParam` | RAM runtime parameters |
| `~/interface_param` | `InterfaceParam` | Communication interface configuration |
| `~/uart_interface_param` | `UartInterfaceParam` | UART parameters |
| `~/iic_interface_param` | `IicInterfaceParam` | IIC parameters |
| `~/uwb_interface_param` | `UwbInterfaceParam` | UWB parameters |
| `~/ble_interface_param` | `BleInterfaceParam` | BLE parameters |
| `~/user_data_from_device` | `UserData` | User-defined data reported by the device |

**User → Device** (publish to send commands)

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/find` | `Find` | Make the device vibrate/blink to locate it |
| `~/restart` | `Restart` | Restart the device |
| `~/state_control` | `StateControl` | Put the tag to sleep or wake it |
| `~/z_measurement` | `ZMeasurement` | Supply an accurate tag z coordinate |
| `~/map_measurement` | `MapMeasurement` | Supply the map the tag is on |
| `~/user_data_to_device` | `UserData` | Send user-defined data |
| `~/param_read` | `std_msgs/Empty` | Read ROM parameters |
| `~/param_write` | `Param` | Write ROM parameters |
| `~/run_time_param_read` | `std_msgs/Empty` | Read RAM runtime parameters |
| `~/run_time_param_write` | `RunTimeParam` | Write RAM runtime parameters |
| `~/interface_param_read` | `std_msgs/Empty` | Read interface configuration |
| `~/uart_interface_param_read` | `std_msgs/Empty` | Read UART parameters |
| `~/iic_interface_param_read` | `std_msgs/Empty` | Read IIC parameters |
| `~/uwb_interface_param_read` | `std_msgs/Empty` | Read UWB parameters |
| `~/ble_interface_param_read` | `std_msgs/Empty` | Read BLE parameters |

## Foxglove Visualization (no ROS required)

On top of its existing log output, `app/reader/` can host a Foxglove WebSocket server. It does not depend on ROS, which makes it handy for inspecting data on machines without a ROS installation. **It is on by default in non-ROS builds**:

```bash
cmake -B build && cmake --build build
./build/app/reader/reader --port /dev/ttyUSB0   # baudrate defaults to 115200, Foxglove port to 8765
./build/app/reader/reader --port /dev/ttyUSB0 --baudrate 115200 --foxglove_port 8765
```

Then in Foxglove choose **Open connection → Foxglove WebSocket** and enter `ws://<device-ip>:8765`. Log printing is unchanged; both run side by side.

| Topic | Type | Purpose |
| --- | --- | --- |
| `/tag_pose` | `foxglove.PoseInFrame` | Tag position, shown in the 3D panel |
| `/scene` | `foxglove.SceneUpdate` | Uncertainty sphere (2σ diameter) + velocity vector + anchors with known coordinates |
| `/tf` | `foxglove.FrameTransform` | `world` → `map`, the frame for the 3D panel |
| `/location_result` | JSON | Position/velocity/noise/error_code, for Plot panels |
| `/anchor_signal` | JSON | Anchor signal strength / packet rate, for Plot panels |
| `/heartbeat` | JSON | Battery and other status, for Plot panels |

Anchor coordinates come from `UB_MSG_ANCHOR_POS`; they are cached and drawn into the 3D scene. Anchors that participated in the current fix are green, the rest grey.

### Data Recording (MCAP)

Add `--record` to record every topic in the table above into an MCAP file that you can drag straight into Foxglove for replay:

```bash
./build/app/reader/reader --port /dev/ttyUSB0 --record
```

The file is written to `logs/ubeacon_<datetime>.mcap` (same directory as the logs, e.g. `logs/ubeacon_20260713_182432_123.mcap`, millisecond precision, so repeated runs never overwrite each other). Recording and the WebSocket server are independent and can be used together.

> Recording requires the Foxglove SDK, i.e. `UB_BUILD_FOXGLOVE=ON` (on by default in non-ROS builds). Exit with `Ctrl+C` so the program can write the index and close the file; a hard kill (`kill -9`) leaves the MCAP without an index and it cannot be opened.

## Serial Port Permissions (Linux)

The first time you access a serial port you often see:

```text
could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0'
```

Serial devices belong to the `dialout` group (`uucp` on some distributions) and regular users are not in it by default. Confirm the group name with `ls -l /dev/ttyUSB0`, add your user to it, and **log out and back in** for it to take effect:

```bash
sudo usermod -a -G dialout $USER   # -a is required, otherwise you are removed from your other groups
```

Reference: [Fix Serial Port “Permission Denied” Errors on Linux](https://websistent.com/fix-serial-port-permission-denied-errors-linux/)

## Build Options (CMake)

| Option | Default | Description |
| --- | --- | --- |
| `UB_BUILD_ROS1` | auto-detected from `ROS_VERSION` | Build the ROS1 driver package |
| `UB_BUILD_ROS2` | auto-detected from `ROS_VERSION` | Build the ROS2 driver package |
| `UB_BUILD_READER` | ON for non-ROS builds / OFF for ROS builds | Build the serial receive-and-parse example |
| `UB_BUILD_WRITER` | ON for non-ROS builds / OFF for ROS builds | Build the encode-and-send example |
| `UB_BUILD_FOXGLOVE` | ON for non-ROS builds / OFF for ROS builds | Enable Foxglove visualization for reader (fetches the Foxglove SDK automatically) |
| `UB_BUILD_TEST` | ON for non-ROS builds / OFF for ROS builds | Build the pure C driver unit tests (fetches Catch2 automatically) |

When no ROS option is given explicitly, the version is chosen from the `ROS_VERSION` environment variable, so `catkin_make` / `colcon build` just work inside a ROS workspace.

Once ROS1 or ROS2 is enabled, the last four options default to OFF: a ROS build produces only the `ubeacon_driver` node and does not additionally compile the host examples or unit tests, nor fetch Catch2 / the Foxglove SDK. Turn any of them back on explicitly if you need it, for example to build and run the unit tests inside a ROS workspace:

```bash
catkin_make -DUB_BUILD_TEST=ON                              # ROS1
colcon build --cmake-args -DUB_BUILD_TEST=ON                # ROS2
```

Outside ROS you can also use the presets in `CMakePresets.json` (Ninja, covering windows/linux/macos × x64/arm64 × debug/release):

```bash
cmake --preset linux-x64-release
cmake --build build/linux-x64-release
ctest --test-dir build/linux-x64-release --output-on-failure
```
