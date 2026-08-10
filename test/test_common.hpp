#pragma once

// 测试只包含公开头文件，以此证明公开 API 足够自足（无需触碰 src/ 下任何内部细节）

#include "ubeacon_driver_for_dev.h"
#include "ubeacon_driver_for_user.h"

#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <vector>

// 捕获解析器回调输出
struct Capture {
  int begin_count = 0;
  int end_count = 0;
  std::vector<uint8_t> uid;
  ub_frame_id_t frame_id = 0;
  struct Msg {
    ub_msg_id_t id = 0;
    std::vector<uint8_t> data;
  };
  std::vector<Msg> msgs;
};

// 主机侧只认自己所接的 tag（gateway/anchor 的上行帧一律丢弃）
inline bool on_begin_from_dev(void *arg, const uint8_t *uid,
                              ub_frame_id_t frame_id) {
  auto *c = static_cast<Capture *>(arg);
  c->begin_count++;
  c->uid.assign(uid, uid + UB_UID_SIZE);
  c->frame_id = frame_id;
  return frame_id == UB_FRAME_ID_TAG_UP;
}

inline bool on_begin_from_user(void *arg, ub_frame_id_t frame_id) {
  auto *c = static_cast<Capture *>(arg);
  c->begin_count++;
  c->frame_id = frame_id;
  return frame_id == UB_FRAME_ID_DOWN;
}

inline void on_end(void *arg) { static_cast<Capture *>(arg)->end_count++; }

inline void on_msg(void *arg, ub_msg_id_t id, const void *data, int data_size) {
  auto *c = static_cast<Capture *>(arg);
  Capture::Msg m;
  m.id = id;
  if (data != nullptr && data_size > 0) {
    const auto *p = static_cast<const uint8_t *>(data);
    m.data.assign(p, p + data_size);
  }
  c->msgs.push_back(std::move(m));
}

inline const uint8_t kUid[UB_UID_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

// 手工拼装一个上行帧，用于精确控制 payload 字节（版本兼容、越界钳位等测试）
inline std::vector<uint8_t>
make_up_frame(ub_msg_id_t id, const std::vector<uint8_t> &payload,
              ub_frame_id_t frame_id = UB_FRAME_ID_TAG_UP) {
  std::vector<uint8_t> f;
  f.push_back(UB_FRAME_SOF);
  const auto payload_size =
      static_cast<uint16_t>(UB_UID_SIZE + 1 + 2 + payload.size());
  f.push_back(static_cast<uint8_t>(payload_size & 0xFF));
  f.push_back(static_cast<uint8_t>(payload_size >> 8));
  f.insert(f.end(), kUid, kUid + UB_UID_SIZE);
  f.push_back(frame_id);
  f.push_back(id);
  f.push_back(static_cast<uint8_t>(payload.size() & 0x7F));
  f.insert(f.end(), payload.begin(), payload.end());
  uint8_t sum = 0;
  for (uint8_t b : f) {
    sum += b;
  }
  f.push_back(sum);
  return f;
}

// 解析一个上行帧
inline Capture parse_from_dev(const std::vector<uint8_t> &bytes) {
  Capture cap;
  UBParserFromDev parser;
  ub_parser_from_dev_init(&parser, on_begin_from_dev, on_msg, on_end, &cap);
  ub_parser_from_dev_handle_data(&parser, bytes.data(),
                                 static_cast<int>(bytes.size()));
  return cap;
}

// 下行：主机构造 -> 设备解析 -> 用解出的结构重新编码，要求与原帧逐字节一致。
// 逐字节一致可以证明 to_raw / from_raw 互为逆运算，不会丢失或错位任何字段。
template <typename T> void roundtrip_to_dev(ub_msg_id_t id, const T &data) {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_dev(id, &data, frame, sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromUser parser;
  ub_parser_from_user_init(&parser, on_begin_from_user, on_msg, on_end, &cap);
  ub_parser_from_user_handle_data(&parser, frame, n);

  REQUIRE(cap.begin_count == 1);
  REQUIRE(cap.end_count == 1);
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == id);
  REQUIRE(cap.msgs[0].data.size() == sizeof(T));

  T decoded;
  std::memcpy(&decoded, cap.msgs[0].data.data(), sizeof(T));
  uint8_t frame2[UB_FRAME_SIZE_MAX];
  const int n2 = ub_prepare_msg_to_dev(id, &decoded, frame2, sizeof(frame2));
  REQUIRE(n2 == n);
  REQUIRE(std::memcmp(frame, frame2, static_cast<size_t>(n)) == 0);
}

// 上行：设备构造 -> 主机解析 -> 重新编码，要求逐字节一致
template <typename T> void roundtrip_to_user(ub_msg_id_t id, const T &data) {
  uint8_t frame[UB_FRAME_SIZE_MAX];
  const int n = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, id, &data,
                                       frame, sizeof(frame));
  REQUIRE(n > 0);

  Capture cap;
  UBParserFromDev parser;
  ub_parser_from_dev_init(&parser, on_begin_from_dev, on_msg, on_end, &cap);
  ub_parser_from_dev_handle_data(&parser, frame, n);

  REQUIRE(cap.begin_count == 1);
  REQUIRE(cap.end_count == 1);
  REQUIRE(cap.uid == std::vector<uint8_t>(kUid, kUid + UB_UID_SIZE));
  REQUIRE(cap.msgs.size() == 1);
  REQUIRE(cap.msgs[0].id == id);
  REQUIRE(cap.msgs[0].data.size() == sizeof(T));

  T decoded;
  std::memcpy(&decoded, cap.msgs[0].data.data(), sizeof(T));
  uint8_t frame2[UB_FRAME_SIZE_MAX];
  const int n2 = ub_prepare_msg_to_user(kUid, UB_FRAME_ID_TAG_UP, id, &decoded,
                                        frame2, sizeof(frame2));
  REQUIRE(n2 == n);
  REQUIRE(std::memcmp(frame, frame2, static_cast<size_t>(n)) == 0);
}

// 上下行都验一遍
template <typename T> void roundtrip(ub_msg_id_t id, const T &data) {
  roundtrip_to_dev(id, data);
  roundtrip_to_user(id, data);
}
