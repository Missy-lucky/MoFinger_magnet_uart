/**
 * @file bsp_i2c.h
 * @brief 板级 I2C 事务抽象，对驱动层隐藏 HAL 句柄。
 * @details 所属层级：BSP。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdint.h>
#include "app_status.h"

/** @brief 向指定 7 位地址执行一次阻塞式主机写事务。 */
AppStatus BspI2c_Write(uint8_t addr7, const uint8_t *data, uint16_t size);

/**
 * @brief 发送一个命令字节，随后以 repeated START 读取响应。
 * @note 用于 MLX90393 的 EX、SM、RM 等命令事务。
 */
AppStatus BspI2c_CommandRead(uint8_t addr7, uint8_t command, uint8_t *rx, uint16_t rx_size);

/**
 * @brief 发送一个命令字节，随后以 repeated START 读取响应，可指定运行期短超时。
 * @note 1000 Hz burst 读取路径使用短超时，避免异常器件长时间阻塞主循环。
 */
AppStatus BspI2c_CommandReadTimeout(uint8_t addr7, uint8_t command, uint8_t *rx,
                                    uint16_t rx_size, uint32_t timeout_ms);

/**
 * @brief 发送“命令+参数”两个字节，随后以 repeated START 读取响应。
 * @note 使用 HAL 16 位 memory-address 事务产生所需波形，主要用于 MLX90393 RR。
 */
AppStatus BspI2c_CommandAddressRead(uint8_t addr7, uint8_t command, uint8_t argument,
                                    uint8_t *rx, uint16_t rx_size);

/**
 * @brief 发送任意长度命令段，并在不产生 STOP 的情况下读取一个状态字节。
 * @note 使用 I2C 中断顺序事务，主要用于 MLX90393 WR；内部带超时和 abort 恢复。
 */
AppStatus BspI2c_CommandWriteReadStatus(uint8_t addr7, const uint8_t *tx, uint16_t tx_size,
                                        uint8_t *status);

/** @brief 按 8 位寄存器地址执行阻塞式 memory read。 */
AppStatus BspI2c_MemRead(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t size);

/** @brief 按 8 位寄存器地址执行阻塞式 memory write。 */
AppStatus BspI2c_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t size);

#endif

