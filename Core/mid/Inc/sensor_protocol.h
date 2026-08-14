/**
 * @file sensor_protocol.h
 * @brief 原始传感器上行协议的快照结构、可选载荷和编码接口。
 * @details 所属层级：MID。当前默认只传输三颗 MLX90393 的原始值，
 *          不在 MCU 内执行姿态识别或磁干扰算法。
 */

#ifndef SENSOR_PROTOCOL_H
#define SENSOR_PROTOCOL_H

#include <stdint.h>
#include "board_config.h"
#include "lsm6dsow.h"
#include "mlx90393.h"

/*
 * 编译期输出选择：1 表示启用，0 表示禁用。
 * 指尖磁铁方案当前不输出 IMU；指腹磁铁方案硬件上没有 IMU，也不能打开 IMU 上报。
 * 两个宏都为 1 时保持原来的 68 字节帧，仅用于指尖硬件后续调试。
 */
#ifndef SENSOR_OUTPUT_ENABLE_MLX
#define SENSOR_OUTPUT_ENABLE_MLX 1u
#endif
#ifndef SENSOR_OUTPUT_ENABLE_IMU
#define SENSOR_OUTPUT_ENABLE_IMU 0u
#endif

#if ((SENSOR_OUTPUT_ENABLE_MLX == 0u) && (SENSOR_OUTPUT_ENABLE_IMU == 0u))
#error "At least one sensor output must be enabled"
#endif

#if ((BOARD_ENABLE_USART1_IN != 0u) && (SENSOR_OUTPUT_ENABLE_IMU != 0u))
#error "指腹磁铁方案没有 IMU，不能开启 SENSOR_OUTPUT_ENABLE_IMU"
#endif

/** @brief 协议固定双字节帧头和单字节帧尾。 */
#define SENSOR_PROTOCOL_HEADER_0 0x1Au
#define SENSOR_PROTOCOL_HEADER_1 0x2Bu
#define SENSOR_PROTOCOL_TAIL 0x3Cu

/** @brief 帧头、传感器域、传感器 id、类型、状态、长度、时间戳、载荷长度、CRC 和帧尾的总开销。 */
#define SENSOR_PROTOCOL_FRAME_OVERHEAD 16u
/** @brief 协议允许转发的最小帧长，等于只有协议头尾且没有载荷。 */
#define SENSOR_PROTOCOL_MIN_FRAME_SIZE SENSOR_PROTOCOL_FRAME_OVERHEAD

/** @brief 根据编译期输出选择计算当前帧的原始载荷长度。 */
#define SENSOR_PROTOCOL_PAYLOAD_SIZE \
  ((SENSOR_OUTPUT_ENABLE_MLX * BOARD_MLX_COUNT * 8u) + \
   (SENSOR_OUTPUT_ENABLE_IMU * BOARD_IMU_COUNT * 14u))

/** @brief 根据编译期输出选择计算当前帧总长度。 */
#define SENSOR_PROTOCOL_FRAME_SIZE (SENSOR_PROTOCOL_FRAME_OVERHEAD + SENSOR_PROTOCOL_PAYLOAD_SIZE)

/** @brief 协议编码所需的一次原始传感器快照。 */
typedef struct {
  uint32_t tick_ms;
  uint8_t gain_sel;
#if SENSOR_OUTPUT_ENABLE_MLX
  Mlx90393Sample mlx[BOARD_MLX_COUNT];
#endif
#if SENSOR_OUTPUT_ENABLE_IMU
  Lsm6dsowSample imu[BOARD_IMU_COUNT];
#endif
} SensorProtocolInput;

/**
 * @brief 将启用的原始传感器快照编码为大端二进制帧。
 * @param input 待编码的原始传感器快照。
 * @param output 输出缓冲区。
 * @param capacity 输出缓冲区容量，至少为 SENSOR_PROTOCOL_FRAME_SIZE。
 * @return 成功返回当前模式的帧长；参数错误、空间不足或偏移异常时返回 0。
 * @details 载荷由 SENSOR_OUTPUT_ENABLE_MLX / SENSOR_OUTPUT_ENABLE_IMU 选择；
 *          CRC 使用单字节 CRC8-CCITT。
 */
uint16_t SensorProtocol_Encode(const SensorProtocolInput *input, uint8_t *output,
                               uint16_t capacity);

/**
 * @brief 校验一帧二进制上行帧是否满足帧头、总长度、载荷长度、CRC 和帧尾约束。
 * @param frame 待校验的完整帧。
 * @param length 待校验帧长。
 * @return 1 表示合法，0 表示非法。
 */
uint8_t SensorProtocol_IsValidFrame(const uint8_t *frame, uint16_t length);

#endif
