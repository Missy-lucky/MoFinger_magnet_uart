/**
 * @file mlx90393.h
 * @brief MLX90393 驱动上下文、原始样本和公开接口。
 * @details 所属层级：DRI。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef MLX90393_H
#define MLX90393_H

#include <stdint.h>
#include "app_status.h"

/** @brief MLX90393 可调整的采样配置，字段值与芯片寄存器位域一致。 */
typedef struct {
  uint8_t gain_sel;
  uint8_t hallconf;
  uint8_t res_x;
  uint8_t res_y;
  uint8_t res_z;
  uint8_t dig_filt;
  uint8_t osr;
  uint8_t osr2;
  uint8_t tcmp_en;
  uint8_t burst_sel;
  uint8_t burst_data_rate;
} Mlx90393Config;

/** @brief 当前配置下 XYZ 三轴的标称磁场灵敏度，单位 uT/LSB。 */
typedef struct {
  float x_ut_per_lsb;
  float y_ut_per_lsb;
  float z_ut_per_lsb;
} Mlx90393Sensitivity;

/** @brief 单颗 MLX90393 的驱动上下文；器件角色由应用层决定。 */
typedef struct {
  uint8_t address;
  uint8_t initialized;
  uint8_t last_init_step;
  uint8_t last_status;
  Mlx90393Config config;
} Mlx90393;

/** @brief MLX90393 的 XYZT 结果及命令状态字节；temperature 为减去 46244 后的原始差值。 */
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t temperature;
  uint8_t status;
} Mlx90393Sample;

/**
 * @brief 复位器件、配置 Reg0/Reg1/Reg2 并逐项回读校验。
 * @param device 驱动上下文。
 * @param address 7 位 I2C 地址。
 * @param config 目标采样配置，初始化成功后复制到驱动上下文。
 */
AppStatus Mlx90393_Init(Mlx90393 *device, uint8_t address,
                        const Mlx90393Config *config);

/**
 * @brief 根据驱动上下文中的 HALLCONF、GAIN_SEL 和各轴 RES 查询灵敏度。
 * @param device 已配置的驱动上下文。
 * @param sensitivity 输出三轴灵敏度。
 * @return 查询成功返回 APP_STATUS_OK；不支持的 HALLCONF 或参数错误返回 BAD_PARAM。
 * @note 当前原始上报链路不调用该接口换算物理量，接口用于配置检查和后续扩展。
 */
AppStatus Mlx90393_GetSensitivity(const Mlx90393 *device,
                                  Mlx90393Sensitivity *sensitivity);

/** @brief 启动一次温度和 XYZ 三轴单次转换。 */
AppStatus Mlx90393_StartSingle(Mlx90393 *device);

/** @brief 启动温度和 XYZ 三轴连续 burst 转换。 */
AppStatus Mlx90393_StartBurst(Mlx90393 *device);

/** @brief 读取最近一次完成的 XYZT 测量结果。 */
AppStatus Mlx90393_Read(Mlx90393 *device, Mlx90393Sample *sample);

/** @brief 读取一项 16 位易失寄存器，并检查 MLX 状态错误位。 */
AppStatus Mlx90393_ReadRegister(Mlx90393 *device, uint8_t reg, uint16_t *value);

/** @brief 写入一项 16 位易失寄存器，并读取命令状态字节。 */
AppStatus Mlx90393_WriteRegister(Mlx90393 *device, uint8_t reg, uint16_t value);

#endif

