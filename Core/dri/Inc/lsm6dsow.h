/**
 * @file lsm6dsow.h
 * @brief LSM6DSOW 驱动上下文、六轴样本和公开接口。
 * @details 所属层级：DRI。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef LSM6DSOW_H
#define LSM6DSOW_H

#include <stdint.h>
#include "app_status.h"

/** @brief LSM6DSOW 运行配置：三个字段即 CTRL1_XL/CTRL2_G/CTRL3_C 的寄存器字节。 */
typedef struct {
  uint8_t ctrl1_xl;
  uint8_t ctrl2_g;
  uint8_t ctrl3_c;
} Lsm6dsowConfig;

/** @brief 单颗 LSM6DSOW 的驱动上下文。 */
typedef struct {
  uint8_t address;
  uint8_t initialized;
} Lsm6dsow;

/** @brief LSM6DSOW 一次同步读取的温度、三轴陀螺和三轴加速度原始值。 */
typedef struct {
  int16_t temperature;
  int16_t gyro[3];
  int16_t accel[3];
} Lsm6dsowSample;

/**
 * @brief 检查 WHO_AM_I 并按传入配置写入 CTRL1_XL/CTRL2_G/CTRL3_C。
 * @param device 驱动上下文。
 * @param address 7 位 I2C 地址。
 * @param config 三个 CTRL 寄存器字节；量程/ODR 等语义集中在 sensor_config.h。
 * @return 初始化结果；ID 不为 0x6C 时返回 APP_STATUS_BAD_ID。
 */
AppStatus Lsm6dsow_Init(Lsm6dsow *device, uint8_t address,
                        const Lsm6dsowConfig *config);

/**
 * @brief 从 OUT_TEMP_L 开始连续读取温度、陀螺和加速度原始数据。
 * @param device 已成功初始化的驱动上下文。
 * @param sample 输出样本。
 */
AppStatus Lsm6dsow_Read(Lsm6dsow *device, Lsm6dsowSample *sample);

#endif

