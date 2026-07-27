#include "ubeacon_frame.h"
#include "ubeacon_checksum.h"
#include <string.h>

// ---------------- 帧解析 ----------------

void ub_frame_buffer_init(UBFrameBuffer *fb) {
  fb->index_begin = 0;
  fb->index_end = 0;
}

static inline int frame_buffer_size(const UBFrameBuffer *fb) {
  return fb->index_end - fb->index_begin;
}

static inline uint8_t *frame_buffer_at(UBFrameBuffer *fb, int index) {
  return fb->buffer + fb->index_begin + index;
}

// 追加一批数据（调用者保证不超出缓冲），随后尽可能多地提取完整帧
static void frame_buffer_append(UBFrameBuffer *fb, const uint8_t *data,
                                int data_size, ub_frame_cb_f cb, void *arg) {
  memcpy(fb->buffer + fb->index_end, data, data_size);
  fb->index_end += data_size;

  while (frame_buffer_size(fb) >= UB_FRAME_SIZE_MIN) {
    if (*frame_buffer_at(fb, 0) != UB_FRAME_SOF) {
      fb->index_begin += 1; // 重同步
      continue;
    }
    int payload_size = ub_frame_get_payload_size(frame_buffer_at(fb, 0));
    if (payload_size > UB_FRAME_PAYLOAD_SIZE_MAX) {
      fb->index_begin += 1;
      continue;
    }
    int frame_size = UB_FRAME_SIZE_MIN + payload_size;
    if (frame_buffer_size(fb) < frame_size) {
      break; // 帧还没收全，等待更多数据
    }
    if (!ub_checksum_verify(frame_buffer_at(fb, 0), frame_size)) {
      fb->index_begin += 1;
      continue;
    }
    if (cb) {
      cb(arg, frame_buffer_at(fb, UB_FRAME_PAYLOAD_OFFSET), payload_size);
    }
    fb->index_begin += frame_size;
  }

  // 压缩缓冲，为后续数据腾出空间
  if (fb->index_begin > 0) {
    memmove(fb->buffer, fb->buffer + fb->index_begin, frame_buffer_size(fb));
    fb->index_end -= fb->index_begin;
    fb->index_begin = 0;
  }
}

void ub_frame_buffer_feed(UBFrameBuffer *fb, const void *data, int data_size,
                          ub_frame_cb_f cb, void *arg) {
  const uint8_t *p = (const uint8_t *)data;
  int offset = 0;
  while (offset < data_size) {
    int space = UB_FRAME_SIZE_MAX - fb->index_end;
    if (space <= 0) {
      // 理论上不会发生：单帧最大长度即为缓冲长度，上面的循环总能取得进展。
      // 兜底丢弃缓冲，避免死循环。
      ub_frame_buffer_init(fb);
      space = UB_FRAME_SIZE_MAX;
    }
    int remain = data_size - offset;
    int size = space < remain ? space : remain;
    frame_buffer_append(fb, p + offset, size, cb, arg);
    offset += size;
  }
}

// ---------------- 帧构造 ----------------

int ub_frame_get_payload_size(const void *frame) {
  ub_frame_payload_size_t payload_size = 0;
  memcpy(&payload_size, (const uint8_t *)frame + sizeof(ub_frame_sof_t),
         sizeof(payload_size));
  return (int)payload_size;
}

void ub_frame_set_payload_size(void *frame, int payload_size) {
  ub_frame_payload_size_t v = (ub_frame_payload_size_t)payload_size;
  memcpy((uint8_t *)frame + sizeof(ub_frame_sof_t), &v, sizeof(v));
}

bool ub_frame_write_begin(void *frame, int frame_size_max) {
  if (frame_size_max < UB_FRAME_SIZE_MIN) {
    return false;
  }
  *(uint8_t *)frame = UB_FRAME_SOF;
  ub_frame_set_payload_size(frame, 0);
  return true;
}

int ub_frame_write_end(void *frame, int frame_size_max) {
  int frame_size = UB_FRAME_SIZE_MIN + ub_frame_get_payload_size(frame);
  if (frame_size > frame_size_max) {
    return -1;
  }
  ub_checksum_update(frame, frame_size);
  return frame_size;
}
