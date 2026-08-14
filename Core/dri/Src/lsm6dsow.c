/**
 * @file lsm6dsow.c
 * @brief LSM6DSOW 身份检查、工作模式配置和温度/六轴读取实现。
 * @details 所属层级：DRI。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "lsm6dsow.h"

#include <stddef.h>

#include "bsp_i2c.h"

#define LSM_REG_WHO_AM_I 0x0Fu
#define LSM_WHO_AM_I_VALUE 0x6Cu
#define LSM_REG_CTRL1_XL 0x10u
#define LSM_REG_CTRL2_G 0x11u
#define LSM_REG_CTRL3_C 0x12u
#define LSM_REG_OUT_TEMP_L 0x20u

/** @brief 实现器件身份检查和基础运行寄存器配置。 */
AppStatus Lsm6dsow_Init(Lsm6dsow *device, uint8_t address,
                        const Lsm6dsowConfig *config) {
  uint8_t id;
  uint8_t value;
  AppStatus result;
  if ((device == NULL) || (config == NULL)) return APP_STATUS_BAD_PARAM;
  device->address = address;
  device->initialized = 0u;
  /* 先确认芯片身份，避免向同地址的其他器件写入配置寄存器。 */
  result = BspI2c_MemRead(address, LSM_REG_WHO_AM_I, &id, 1u);
  if (result != APP_STATUS_OK) return result;
  if (id != LSM_WHO_AM_I_VALUE) return APP_STATUS_BAD_ID;
  /* 加速度计 ODR/量程：由 sensor_config.h 的宏组合而成。 */
  value = config->ctrl1_xl;
  result = BspI2c_MemWrite(address, LSM_REG_CTRL1_XL, &value, 1u);
  if (result != APP_STATUS_OK) return result;
  /* 陀螺仪 ODR/量程：由 sensor_config.h 的宏组合而成。 */
  value = config->ctrl2_g;
  result = BspI2c_MemWrite(address, LSM_REG_CTRL2_G, &value, 1u);
  if (result != APP_STATUS_OK) return result;
  /* 公共控制：BDU 保证高低字节一致，IF_INC 支持 14 字节连续读取。 */
  value = config->ctrl3_c;
  result = BspI2c_MemWrite(address, LSM_REG_CTRL3_C, &value, 1u);
  if (result == APP_STATUS_OK) device->initialized = 1u;
  return result;
}

/** @brief 实现 14 字节 burst read，并按小端格式还原温度和六轴 int16_t。 */
AppStatus Lsm6dsow_Read(Lsm6dsow *device, Lsm6dsowSample *sample) {
  uint8_t raw[14];
  AppStatus result;
  if ((device == NULL) || (sample == NULL)) return APP_STATUS_BAD_PARAM;
  if (device->initialized == 0u) return APP_STATUS_NOT_READY;
  result = BspI2c_MemRead(device->address, LSM_REG_OUT_TEMP_L, raw, sizeof(raw));
  if (result != APP_STATUS_OK) return result;

  /* OUT_TEMP_L/H 位于连续输出区首部，先还原温度原始码。 */
  sample->temperature = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);

  /* 后续寄存器仍为小端序：依次还原陀螺三轴和加速度三轴。 */
  for (uint8_t axis = 0u; axis < 3u; ++axis) {
    sample->gyro[axis] =
        (int16_t)(((uint16_t)raw[axis * 2u + 3u] << 8) | raw[axis * 2u + 2u]);
    sample->accel[axis] =
        (int16_t)(((uint16_t)raw[axis * 2u + 9u] << 8) | raw[axis * 2u + 8u]);
  }
  return APP_STATUS_OK;
}







