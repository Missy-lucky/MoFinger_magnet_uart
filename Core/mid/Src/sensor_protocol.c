/**
 * @file sensor_protocol.c
 * @brief 指尖传感器原始数据的大端序列化和 CRC8 封装。
 * @details 所属层级：MID。编码过程显式写入每个字段，不依赖结构体对齐方式，
 *          也不在 MCU 内执行姿态识别、磁干扰判断或单位换算。
 */

#include "sensor_protocol.h"

#include <stddef.h>

#include "com_crc.h"

/** @brief 按大端读取一个 16 位无符号整数。 */
static uint16_t get_u16_be(const uint8_t *buffer, uint16_t offset) {
  return (uint16_t)(((uint16_t)buffer[offset] << 8) | buffer[offset + 1u]);
}

/** @brief 协议固定双字节帧头。 */
#define SENSOR_FRAME_HEADER_0 SENSOR_PROTOCOL_HEADER_0
#define SENSOR_FRAME_HEADER_1 SENSOR_PROTOCOL_HEADER_1
/** @brief 协议固定单字节帧尾。 */
#define SENSOR_FRAME_TAIL SENSOR_PROTOCOL_TAIL
/**
 * @brief 按大端序写入一个 16 位整数并推进偏移。
 * @param buffer 目标缓冲区。
 * @param offset 当前写入偏移，完成后增加 2。
 * @param value 待写入值；有符号原始值转换后保持其二进制补码位模式。
 */
static void put_u16_be(uint8_t *buffer, uint16_t *offset, uint16_t value) {
  buffer[(*offset)++] = (uint8_t)(value >> 8);
  buffer[(*offset)++] = (uint8_t)value;
}

/**
 * @brief 按大端序写入一个 32 位整数并推进偏移。
 * @param buffer 目标缓冲区。
 * @param offset 当前写入偏移，完成后增加 4。
 * @param value 待写入值。
 */
static void put_u32_be(uint8_t *buffer, uint16_t *offset, uint32_t value) {
  buffer[(*offset)++] = (uint8_t)(value >> 24);
  buffer[(*offset)++] = (uint8_t)(value >> 16);
  buffer[(*offset)++] = (uint8_t)(value >> 8);
  buffer[(*offset)++] = (uint8_t)value;
}

/**
 * @brief 根据编译期选择生成原始传感器上行帧。
 * @param input 当前方案启用的原始传感器快照。
 * @param output 调用者提供的帧缓冲区。
 * @param capacity 帧缓冲区容量。
 * @return 成功返回当前模式帧长，否则返回 0。
 */
uint16_t SensorProtocol_Encode(const SensorProtocolInput *input, uint8_t *output,
                               uint16_t capacity) {
  uint16_t offset = 0u;
  uint16_t payload_start;
  uint8_t crc;

  /* 编码前统一检查指针和容量，后续定长写入不再重复判断边界。 */
  if ((input == NULL) || (output == NULL) ||
      (capacity < SENSOR_PROTOCOL_FRAME_SIZE)) {
    return 0u;
  }

  /* 固定协议头：状态字节直接携带当前 MLX90393 GAIN_SEL，便于上位机动态换算。 */
  /* 字段顺序：帧头、传感器域、传感器 id、传感器类型、GAIN_SEL、总帧长。 */
  output[offset++] = SENSOR_FRAME_HEADER_0;
  output[offset++] = SENSOR_FRAME_HEADER_1;
  output[offset++] = BOARD_SENSOR_DOMAIN;
  output[offset++] = BOARD_SENSOR_ID;
  output[offset++] = BOARD_SENSOR_TYPE;
  output[offset++] = (uint8_t)(input->gain_sel & 0x07u);
  put_u16_be(output, &offset, SENSOR_PROTOCOL_FRAME_SIZE);

  /* 时间戳和报文长度均使用大端序；报文长度只统计后续原始载荷。 */
  put_u32_be(output, &offset, input->tick_ms);
  put_u16_be(output, &offset, SENSOR_PROTOCOL_PAYLOAD_SIZE);
  payload_start = offset;

  /* 每颗 MLX 固定按温度、X、Y、Z 顺序写入四个 16 位原始值。 */
#if SENSOR_OUTPUT_ENABLE_MLX
  for (uint8_t i = 0u; i < BOARD_MLX_COUNT; ++i) {
    put_u16_be(output, &offset, (uint16_t)input->mlx[i].temperature);
    put_u16_be(output, &offset, (uint16_t)input->mlx[i].x);
    put_u16_be(output, &offset, (uint16_t)input->mlx[i].y);
    put_u16_be(output, &offset, (uint16_t)input->mlx[i].z);
  }
#endif

  /* 每颗 IMU 固定按加速度 XYZ、角速度 XYZ、温度顺序写入七个原始值。 */
#if SENSOR_OUTPUT_ENABLE_IMU
  for (uint8_t i = 0u; i < BOARD_IMU_COUNT; ++i) {
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
      put_u16_be(output, &offset, (uint16_t)input->imu[i].accel[axis]);
    }
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
      put_u16_be(output, &offset, (uint16_t)input->imu[i].gyro[axis]);
    }
    put_u16_be(output, &offset, (uint16_t)input->imu[i].temperature);
  }
#endif

  /* 载荷偏移检查用于防止后续增删字段时忘记同步协议长度常量。 */
  if ((uint16_t)(offset - payload_start) != SENSOR_PROTOCOL_PAYLOAD_SIZE) {
    return 0u;
  }

  /* CRC8 覆盖帧头至载荷末字节，不包含 CRC 自身和帧尾。 */
  crc = ComCrc8Ccitt_Calculate(output, offset);
  output[offset++] = crc;
  output[offset++] = SENSOR_FRAME_TAIL;

  return (offset == SENSOR_PROTOCOL_FRAME_SIZE) ? offset : 0u;
}

uint8_t SensorProtocol_IsValidFrame(const uint8_t *frame, uint16_t length) {
  uint16_t field_length;
  uint16_t payload_length;
  uint8_t crc;

  if ((frame == NULL) || (length < SENSOR_PROTOCOL_MIN_FRAME_SIZE)) {
    return 0u;
  }
  if ((frame[0] != SENSOR_PROTOCOL_HEADER_0) ||
      (frame[1] != SENSOR_PROTOCOL_HEADER_1) ||
      (frame[length - 1u] != SENSOR_PROTOCOL_TAIL)) {
    return 0u;
  }

  field_length = get_u16_be(frame, 6u);
  payload_length = get_u16_be(frame, 12u);
  if ((field_length != length) ||
      ((uint16_t)(payload_length + SENSOR_PROTOCOL_FRAME_OVERHEAD) != length)) {
    return 0u;
  }

  crc = ComCrc8Ccitt_Calculate(frame, (uint32_t)(length - 2u));
  return (crc == frame[length - 2u]) ? 1u : 0u;
}
