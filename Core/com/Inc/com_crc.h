/**
 * @file com_crc.h
 * @brief 公共层 CRC8 校验接口，供通信协议和持久化数据复用。
 * @details 所属层级：COM。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef COM_CRC_H
#define COM_CRC_H

#include <stdint.h>

/**
 * @brief 计算 CRC8-CCITT 校验值。
 * @param data 待校验数据首地址；当 length 大于 0 时不得为 NULL。
 * @param length 数据字节数。
 * @return CRC8 校验值；参数无效时返回 0。
 * @details 参数为 Poly=0x07、Init=0x00、RefIn=false、RefOut=false、
 *          XorOut=0x00；字符串 "123456789" 的标准校验值为 0xF4。
 */
uint8_t ComCrc8Ccitt_Calculate(const void *data, uint32_t length);

#endif


