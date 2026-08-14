/**
 * @file bsp_i2c.c
 * @brief I2C1 阻塞事务及 MLX 顺序 repeated START 事务实现。
 * @details 所属层级：BSP。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "bsp_i2c.h"

#include "board_config.h"
#include "bsp_handles.h"

/** @brief 将 STM32 HAL 状态统一映射为项目状态码。 */
static AppStatus map_hal(HAL_StatusTypeDef status) {
  if (status == HAL_OK) return APP_STATUS_OK;
  if (status == HAL_BUSY) return APP_STATUS_BUSY;
  if (status == HAL_TIMEOUT) return APP_STATUS_TIMEOUT;
  return APP_STATUS_IO;
}

/**
 * @brief 等待 I2C 中断事务结束，并把超时或外设错误转换为项目状态码。
 * @param addr7 当前事务使用的 7 位从机地址，仅在超时时供 HAL abort 使用。
 * @return 事务正常结束返回 APP_STATUS_OK；超时返回 APP_STATUS_TIMEOUT；
 *         NACK、总线错误、仲裁丢失等 HAL 错误返回 APP_STATUS_IO。
 * @details HAL 的顺序传输在一帧完成后会将句柄状态恢复为 READY。若等待超时，函数先
 *          请求异步 abort，再短暂等待 abort 中断完成，避免下一次事务立即遇到 BUSY/ABORT
 *          状态。即使句柄已回到 READY，也仍检查 ErrorCode，防止把 NACK 错判为成功。
 */
static AppStatus wait_it_transfer(uint8_t addr7) {
  uint32_t started = HAL_GetTick();

  /* 顺序 IT 接口靠 EV/ER 中断推进；主循环只等待句柄回到 READY。 */
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    if ((HAL_GetTick() - started) >= BOARD_I2C_TIMEOUT_MS) {
      /* 超时后主动终止当前地址事务，并给 abort 中断一个有限的收尾窗口。 */
      if (HAL_I2C_Master_Abort_IT(&hi2c1, (uint16_t)(addr7 << 1)) == HAL_OK) {
        uint32_t abort_started = HAL_GetTick();
        /* 顺序 IT 接口靠 EV/ER 中断推进；主循环只等待句柄回到 READY。 */
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
          if ((HAL_GetTick() - abort_started) >= BOARD_I2C_TIMEOUT_MS) break;
        }
      }
      return APP_STATUS_TIMEOUT;
    }
  }

  return (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_NONE) ? APP_STATUS_OK : APP_STATUS_IO;
}

/** @brief 实现普通 I2C 写事务。 */
AppStatus BspI2c_Write(uint8_t addr7, const uint8_t *data, uint16_t size) {
  if ((data == NULL) || (size == 0u)) return APP_STATUS_BAD_PARAM;
  return map_hal(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1),
                                         (uint8_t *)data, size, BOARD_I2C_TIMEOUT_MS));
}

/** @brief 利用 HAL memory-read 生成单命令字节后的 repeated START。 */
AppStatus BspI2c_CommandRead(uint8_t addr7, uint8_t command, uint8_t *rx, uint16_t rx_size) {
  return BspI2c_CommandReadTimeout(addr7, command, rx, rx_size, BOARD_I2C_TIMEOUT_MS);
}

/** @brief 利用 HAL memory-read 生成单命令字节后的 repeated START，并使用指定超时。 */
AppStatus BspI2c_CommandReadTimeout(uint8_t addr7, uint8_t command, uint8_t *rx,
                                    uint16_t rx_size, uint32_t timeout_ms) {
  if ((rx == NULL) || (rx_size == 0u)) return APP_STATUS_BAD_PARAM;
  /* HAL memory-read emits: START, address+W, command, repeated START, address+R. */
  return map_hal(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), command,
                                  I2C_MEMADD_SIZE_8BIT, rx, rx_size, timeout_ms));
}

/** @brief 利用 16 位 memory address 发送命令与参数，再读取响应。 */
AppStatus BspI2c_CommandAddressRead(uint8_t addr7, uint8_t command, uint8_t argument,
                                    uint8_t *rx, uint16_t rx_size) {
  uint16_t command_address = (uint16_t)(((uint16_t)command << 8) | argument);
  if ((rx == NULL) || (rx_size == 0u)) return APP_STATUS_BAD_PARAM;
  return map_hal(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), command_address,
                                  I2C_MEMADD_SIZE_16BIT, rx, rx_size,
                                  BOARD_I2C_TIMEOUT_MS));
}

/** @brief 通过顺序中断 API 完成多字节写段和单字节状态读取。 */
AppStatus BspI2c_CommandWriteReadStatus(uint8_t addr7, const uint8_t *tx, uint16_t tx_size,
                                        uint8_t *status) {
  HAL_StatusTypeDef hal_status;
  AppStatus result;
  if ((tx == NULL) || (tx_size == 0u) || (status == NULL)) return APP_STATUS_BAD_PARAM;

  /* 第一帧使用 SOFTEND，不发 STOP，为随后切换读方向保留总线。 */
  hal_status = HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, (uint16_t)(addr7 << 1),
                                              (uint8_t *)tx, tx_size, I2C_FIRST_FRAME);
  if (hal_status != HAL_OK) return map_hal(hal_status);
  result = wait_it_transfer(addr7);
  if (result != APP_STATUS_OK) return result;

  /* 第二帧生成 repeated START，读一个状态字节后由 LAST_FRAME 自动发 STOP。 */
  hal_status = HAL_I2C_Master_Seq_Receive_IT(&hi2c1, (uint16_t)(addr7 << 1), status, 1u,
                                             I2C_LAST_FRAME);
  if (hal_status != HAL_OK) return map_hal(hal_status);
  return wait_it_transfer(addr7);
}

/** @brief 实现标准 8 位寄存器读取。 */
AppStatus BspI2c_MemRead(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t size) {
  if ((data == NULL) || (size == 0u)) return APP_STATUS_BAD_PARAM;
  return map_hal(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), reg,
                                  I2C_MEMADD_SIZE_8BIT, data, size, BOARD_I2C_TIMEOUT_MS));
}

/** @brief 实现标准 8 位寄存器写入。 */
AppStatus BspI2c_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t size) {
  if ((data == NULL) || (size == 0u)) return APP_STATUS_BAD_PARAM;
  return map_hal(HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr7 << 1), reg,
                                   I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, size,
                                   BOARD_I2C_TIMEOUT_MS));
}






