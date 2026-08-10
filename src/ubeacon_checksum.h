#pragma once

// 内部头文件：帧校验和（8 位累加和），用户不应包含此文件

#ifdef __cplusplus
extern "C" {
#endif

#include "ubeacon_driver_common.h"

typedef uint8_t ub_checksum_t;

// 对 data_size 个字节求 8 位累加和
static inline ub_checksum_t ub_checksum_get(const void *data, int data_size) {
  const uint8_t *p = (const uint8_t *)data;
  ub_checksum_t sum = 0;
  for (int i = 0; i < data_size; ++i) {
    sum += p[i];
  }
  return sum;
}

// data_size 含末尾的校验和字节
static inline bool ub_checksum_verify(const void *data, int data_size) {
  const uint8_t *p = (const uint8_t *)data;
  return p[data_size - 1] == ub_checksum_get(data, data_size - 1);
}

// data_size 含末尾的校验和字节，就地写入校验和
static inline void ub_checksum_update(void *data, int data_size) {
  uint8_t *p = (uint8_t *)data;
  p[data_size - 1] = ub_checksum_get(data, data_size - 1);
}

#ifdef __cplusplus
}
#endif
