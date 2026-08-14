/**
 * @file com_crc.c
 * @brief CRC8-CCITT 的无硬件依赖实现。
 * @details 所属层级：COM。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "com_crc.h"

#include <stddef.h>

/**
 * @brief 逐字节、从 bit7 到 bit0 计算 CRC8-CCITT。
 * @param data 输入字节流。
 * @param length 输入长度。
 * @return CRC8 校验结果；空指针且长度非零时返回 0。
 */
uint8_t ComCrc8Ccitt_Calculate(const void *data, uint32_t length) {
  const uint8_t *bytes = (const uint8_t *)data;
  uint8_t crc = 0x00u;

  if ((data == NULL) && (length != 0u)) return 0u;
  for (uint32_t i = 0u; i < length; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
      crc = ((crc & 0x80u) != 0u)
                ? (uint8_t)((crc << 1) ^ 0x07u)
                : (uint8_t)(crc << 1);
    }
  }
  return crc;
}
