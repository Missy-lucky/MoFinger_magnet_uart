/**
 * @file h503_flash.h
 * @brief H503 保留参数 Sector 的读取和“保留其余数据的局部更新”接口。
 * @details 所属层级：BSP。对外不暴露裸擦除函数，避免调用者清一个标志时误删同 Sector 数据。
 */

#ifndef H503_FLASH_H
#define H503_FLASH_H

#include <stdint.h>
#include "app_status.h"

/** @brief H503 单个 Flash Sector 和本接口所需 RAM 备份区大小，均为 8 KB。 */
#define H503_FLASH_SECTOR_BACKUP_SIZE 0x2000u

/**
 * @brief 从保留参数 Sector 中读取指定范围。
 * @param offset 相对于参数 Sector 起点的字节偏移。
 * @param data 输出 RAM 缓冲区。
 * @param length 读取字节数，可为 0。
 * @return 成功返回 APP_STATUS_OK；范围或指针非法返回 APP_STATUS_BAD_PARAM。
 */
AppStatus H503Flash_Read(uint32_t offset, void *data, uint32_t length);

/**
 * @brief 保留同一 Sector 其余内容，只更新指定范围。
 * @param offset 相对于参数 Sector 起点的字节偏移，不要求 16 字节对齐。
 * @param data 待覆盖的新数据。
 * @param length 更新字节数，必须大于 0 且不能越过 8 KB Sector。
 * @param sector_backup 调用者提供的 8 KB RAM 备份区。
 * @param backup_size 备份区长度，必须至少为 H503_FLASH_SECTOR_BACKUP_SIZE。
 * @return 完整读取、修改、擦除、写回和整区校验成功返回 APP_STATUS_OK。
 * @details 函数先把完整 8 KB 复制到 RAM，在 RAM 中覆盖目标字节，再擦除整个 Sector，最后
 *          以 16 字节 Quad-word 写回并校验完整 Sector。更新期间掉电或复位仍可能丢失参数，
 *          因此调用者应在稳定供电条件下执行；需要掉电安全时应使用双 Sector 事务方案。
 */
AppStatus H503Flash_UpdatePreserve(uint32_t offset, const void *data, uint32_t length,
                                   uint8_t *sector_backup, uint32_t backup_size);

#endif
