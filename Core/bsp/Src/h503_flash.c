/**
 * @file h503_flash.c
 * @brief H503 参数 Sector 的整区备份、局部修改、擦除恢复和完整校验实现。
 * @details 所属层级：BSP。当前目标由 board_config.h 固定为链接脚本保留的最后 8 KB，
 *          地址最终确认后只需同步板级常量、Bank/Sector 映射和各固件链接脚本。
 */

#include "h503_flash.h"

#include <stddef.h>
#include <string.h>
#include "board_config.h"
#include "stm32h5xx_hal.h"

#define H503_FLASH_QUADWORD_SIZE 16u
#define H503_FLASH_CONFIG_BANK FLASH_BANK_2
#define H503_FLASH_CONFIG_SECTOR FLASH_SECTOR_7

/**
 * @brief 检查参数区内的 offset/length，使用减法形式避免无符号加法溢出。
 * @param offset 参数 Sector 内偏移。
 * @param length 操作长度。
 * @return 范围完整位于 8 KB 参数 Sector 内返回 1，否则返回 0。
 */
static uint8_t range_valid(uint32_t offset, uint32_t length) {
  return (length <= BOARD_FLASH_CONFIG_SIZE) && (offset <= (BOARD_FLASH_CONFIG_SIZE - length));
}

/**
 * @brief 擦除固定参数 Sector，并确认整区均恢复为 0xFF。
 * @return 擦除和校验成功返回 APP_STATUS_OK，否则返回 APP_STATUS_IO 或 APP_STATUS_VERIFY。
 */
static AppStatus erase_config_sector(void) {
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0xFFFFFFFFu;
  HAL_StatusTypeDef result;

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = H503_FLASH_CONFIG_BANK;
  erase.Sector = H503_FLASH_CONFIG_SECTOR;
  erase.NbSectors = 1u;

  if (HAL_FLASH_Unlock() != HAL_OK) return APP_STATUS_IO;
  result = HAL_FLASHEx_Erase(&erase, &sector_error);
  (void)HAL_FLASH_Lock();
  if (result != HAL_OK) return APP_STATUS_IO;

  /* 擦除完成后扫描完整 8 KB，避免在半擦除状态下继续恢复数据。 */
  for (uint32_t offset = 0u; offset < BOARD_FLASH_CONFIG_SIZE; offset += sizeof(uint32_t)) {
    const uint32_t *word = (const uint32_t *)(BOARD_FLASH_CONFIG_ADDRESS + offset);
    if (*word != 0xFFFFFFFFu) return APP_STATUS_VERIFY;
  }
  return APP_STATUS_OK;
}

/**
 * @brief 将完整 8 KB RAM 镜像按 H5 Quad-word 单元写入已擦除的参数 Sector。
 * @param image 指向至少 8 KB 且可读的 RAM 镜像。
 * @return 写入成功返回 APP_STATUS_OK，HAL 编程失败返回 APP_STATUS_IO。
 */
static AppStatus write_config_sector(const uint8_t *image) {
  HAL_StatusTypeDef result = HAL_OK;

  if (HAL_FLASH_Unlock() != HAL_OK) return APP_STATUS_IO;
  for (uint32_t offset = 0u; offset < BOARD_FLASH_CONFIG_SIZE; offset += H503_FLASH_QUADWORD_SIZE) {
    /* HAL 要求 DataAddress 至少 32 位对齐；调用者备份区地址可能不满足，因此先复制到对齐缓冲。 */
    uint32_t quadword[H503_FLASH_QUADWORD_SIZE / sizeof(uint32_t)];
    memcpy(quadword, image + offset, H503_FLASH_QUADWORD_SIZE);
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                               BOARD_FLASH_CONFIG_ADDRESS + offset,
                               (uint32_t)quadword);
    if (result != HAL_OK) break;
  }
  (void)HAL_FLASH_Lock();
  return (result == HAL_OK) ? APP_STATUS_OK : APP_STATUS_IO;
}

/** @brief 通过存储器映射把参数区指定范围复制到 RAM。 */
AppStatus H503Flash_Read(uint32_t offset, void *data, uint32_t length) {
  if (!range_valid(offset, length)) return APP_STATUS_BAD_PARAM;
  if ((data == NULL) && (length != 0u)) return APP_STATUS_BAD_PARAM;
  if (length != 0u) {
    memcpy(data, (const void *)(BOARD_FLASH_CONFIG_ADDRESS + offset), length);
  }
  return APP_STATUS_OK;
}

/** @brief 实现完整 Sector 的读、改、擦、写和整区比较事务。 */
AppStatus H503Flash_UpdatePreserve(uint32_t offset, const void *data, uint32_t length,
                                   uint8_t *sector_backup, uint32_t backup_size) {
  AppStatus result;

  if ((data == NULL) || (sector_backup == NULL) || (length == 0u)) return APP_STATUS_BAD_PARAM;
  if ((backup_size < H503_FLASH_SECTOR_BACKUP_SIZE) || !range_valid(offset, length)) {
    return APP_STATUS_BAD_PARAM;
  }

  /* 第一步：在任何破坏性操作前保存完整 Sector，包括系统参数和校准参数。 */
  memcpy(sector_backup, (const void *)BOARD_FLASH_CONFIG_ADDRESS, BOARD_FLASH_CONFIG_SIZE);

  /* 第二步：只在 RAM 镜像中修改调用者指定的标志或字段。 */
  memcpy(sector_backup + offset, data, length);

  /* 如果新内容与 Flash 完全一致，则无需承担擦写寿命和掉电窗口。 */
  if (memcmp((const void *)BOARD_FLASH_CONFIG_ADDRESS,
             sector_backup, BOARD_FLASH_CONFIG_SIZE) == 0) {
    return APP_STATUS_OK;
  }

  /* 第三步：擦除完整 8 KB；此后直到写回完成前存在掉电数据丢失窗口。 */
  result = erase_config_sector();
  if (result != APP_STATUS_OK) return result;

  /* 第四步：恢复修改后的完整 RAM 镜像，而不是只写目标标志。 */
  result = write_config_sector(sector_backup);
  if (result != APP_STATUS_OK) return result;

  /* 第五步：比较完整 8 KB，确保系统参数和校准参数都已正确恢复。 */
  return (memcmp((const void *)BOARD_FLASH_CONFIG_ADDRESS,
                 sector_backup, BOARD_FLASH_CONFIG_SIZE) == 0)
             ? APP_STATUS_OK : APP_STATUS_VERIFY;
}
