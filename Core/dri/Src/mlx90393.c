/**
 * @file mlx90393.c
 * @brief MLX90393 I2C 命令、寄存器配置和单次测量实现。
 * @details 所属层级：DRI。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "mlx90393.h"

#include <stddef.h>

#include "board_config.h"
#include "bsp_i2c.h"
#include "bsp_time.h"

/* 命令字节：低四位 zyxt 全置位时同时操作温度和三轴磁场。 */
#define MLX_CMD_EXIT 0x80u
#define MLX_CMD_RESET 0xF0u
#define MLX_CMD_START_XYZT 0x3Fu
#define MLX_CMD_START_BURST_BASE 0x10u
#define MLX_CMD_READ_XYZT 0x4Fu
#define MLX_CMD_XYZT_MASK 0x0Fu
#define MLX_STATUS_BURST_MODE 0x80u
#define MLX_STATUS_ERROR 0x10u
#define MLX_TEMP_ZERO_RAW 46244u
#define MLX_CMD_READ_REGISTER 0x50u
#define MLX_CMD_WRITE_REGISTER 0x60u

/* 寄存器位域位置统一在驱动内部维护，应用层只填写语义化配置结构体。 */
#define MLX_REG1_TCMP_EN (1u << 10)
#define MLX_REG1_BURST_SEL_LOW_SHIFT 8u
#define MLX_REG1_BURST_SEL_HIGH_SHIFT 6u
#define MLX_REG1_BURST_DATA_RATE_MASK 0x003Fu
#define MLX_REG2_OSR_SHIFT 0u
#define MLX_REG2_DIG_FILT_SHIFT 2u
#define MLX_REG2_RES_X_SHIFT 5u
#define MLX_REG2_RES_Y_SHIFT 7u
#define MLX_REG2_RES_Z_SHIFT 9u
#define MLX_REG2_OSR2_SHIFT 11u

/** @brief 灵敏度表中的一组 XY/Z 标称系数。 */
typedef struct {
  float xy_ut_per_lsb;
  float z_ut_per_lsb;
} Mlx90393SensitivityEntry;

/* HALLCONF=0xC 时，第一维为 GAIN_SEL，第二维为 RES。 */
static const Mlx90393SensitivityEntry g_sensitivity_hallconf_c[8][4] = {
    {{0.751f, 1.210f}, {1.502f, 2.420f}, {3.004f, 4.840f}, {6.009f, 9.680f}},
    {{0.601f, 0.968f}, {1.202f, 1.936f}, {2.403f, 3.872f}, {4.840f, 7.744f}},
    {{0.451f, 0.726f}, {0.901f, 1.452f}, {1.803f, 2.904f}, {3.605f, 5.808f}},
    {{0.376f, 0.605f}, {0.751f, 1.210f}, {1.502f, 2.420f}, {3.004f, 4.840f}},
    {{0.300f, 0.484f}, {0.601f, 0.968f}, {1.202f, 1.936f}, {2.403f, 3.872f}},
    {{0.250f, 0.403f}, {0.501f, 0.807f}, {1.001f, 1.613f}, {2.003f, 3.227f}},
    {{0.200f, 0.323f}, {0.401f, 0.645f}, {0.801f, 1.291f}, {1.602f, 2.581f}},
    {{0.150f, 0.242f}, {0.300f, 0.484f}, {0.601f, 0.968f}, {1.202f, 1.936f}},
};

/* HALLCONF=0x0 使用另一套数据手册标称灵敏度。 */
static const Mlx90393SensitivityEntry g_sensitivity_hallconf_0[8][4] = {
    {{0.787f, 1.267f}, {1.573f, 2.534f}, {3.146f, 5.068f}, {6.292f, 10.137f}},
    {{0.629f, 1.014f}, {1.258f, 2.027f}, {2.517f, 4.055f}, {5.034f, 8.109f}},
    {{0.472f, 0.760f}, {0.944f, 1.521f}, {1.888f, 3.041f}, {3.775f, 6.082f}},
    {{0.393f, 0.634f}, {0.787f, 1.267f}, {1.573f, 2.534f}, {3.146f, 5.068f}},
    {{0.315f, 0.507f}, {0.629f, 1.014f}, {1.258f, 2.027f}, {2.517f, 4.055f}},
    {{0.262f, 0.422f}, {0.524f, 0.845f}, {1.049f, 1.689f}, {2.097f, 3.379f}},
    {{0.210f, 0.338f}, {0.419f, 0.676f}, {0.839f, 1.352f}, {1.678f, 2.703f}},
    {{0.157f, 0.253f}, {0.315f, 0.507f}, {0.629f, 1.014f}, {1.258f, 2.027f}},
};

/** @brief 检查配置字段是否落在寄存器允许范围内。 */
static uint8_t config_valid(const Mlx90393Config *config) {
  if (config == NULL) return 0u;
  if ((config->gain_sel > 7u) || (config->res_x > 3u) ||
      (config->res_y > 3u) || (config->res_z > 3u) ||
      (config->dig_filt > 7u) || (config->osr > 3u) ||
      (config->osr2 > 3u) || (config->tcmp_en > 1u) ||
      (config->burst_sel > 0x0Fu) || (config->burst_data_rate > 0x3Fu)) {
    return 0u;
  }
  return ((config->hallconf == 0x0Cu) || (config->hallconf == 0x00u)) ? 1u : 0u;
}

/** @brief Reject an undriven/all-high RM response before accepting its status byte. */
static uint8_t rm_response_invalid(const uint8_t *rx) {
  uint8_t all_ff = 1u;
  for (uint8_t i = 0u; i < 9u; ++i) {
    if (rx[i] != 0xFFu) {
      all_ff = 0u;
      break;
    }
  }
  return (uint8_t)(all_ff ||
                   ((rx[3] == 0xFFu) && (rx[4] == 0xFFu) &&
                    (rx[5] == 0xFFu) && (rx[6] == 0xFFu) &&
                    (rx[7] == 0xFFu) && (rx[8] == 0xFFu)));
}

/** @brief 根据语义化配置生成 Reg0 低字节配置。 */
static uint16_t build_reg0_config(const Mlx90393Config *config) {
  return (uint16_t)(((uint16_t)config->gain_sel << 4) | config->hallconf);
}

/** @brief 在保留 Reg1 其它位的基础上写入 TCMP、BURST_SEL 和 BDR。 */
static uint16_t build_reg1_config(uint16_t current, const Mlx90393Config *config) {
  uint16_t reg1 = current;
  uint16_t burst_sel = config->burst_sel;

  reg1 = (config->tcmp_en != 0u)
             ? (uint16_t)(reg1 | MLX_REG1_TCMP_EN)
             : (uint16_t)(reg1 & (uint16_t)~MLX_REG1_TCMP_EN);

  reg1 &= (uint16_t)~((uint16_t)(0x03u << MLX_REG1_BURST_SEL_LOW_SHIFT) |
                      (uint16_t)(0x03u << MLX_REG1_BURST_SEL_HIGH_SHIFT) |
                      MLX_REG1_BURST_DATA_RATE_MASK);
  reg1 |= (uint16_t)(((burst_sel & 0x03u) << MLX_REG1_BURST_SEL_LOW_SHIFT) |
                     (((burst_sel >> 2) & 0x03u) << MLX_REG1_BURST_SEL_HIGH_SHIFT) |
                     (config->burst_data_rate & MLX_REG1_BURST_DATA_RATE_MASK));
  return reg1;
}

/** @brief 根据语义化配置生成完整 Reg2。 */
static uint16_t build_reg2_config(const Mlx90393Config *config) {
  return (uint16_t)(((uint16_t)config->osr2 << MLX_REG2_OSR2_SHIFT) |
                    ((uint16_t)config->res_z << MLX_REG2_RES_Z_SHIFT) |
                    ((uint16_t)config->res_y << MLX_REG2_RES_Y_SHIFT) |
                    ((uint16_t)config->res_x << MLX_REG2_RES_X_SHIFT) |
                    ((uint16_t)config->dig_filt << MLX_REG2_DIG_FILT_SHIFT) |
                    ((uint16_t)config->osr << MLX_REG2_OSR_SHIFT));
}

/** @brief 根据设备实际保存的配置查找三轴灵敏度。 */
AppStatus Mlx90393_GetSensitivity(const Mlx90393 *device,
                                  Mlx90393Sensitivity *sensitivity) {
  const Mlx90393SensitivityEntry (*table)[4];
  uint8_t gain;

  if ((device == NULL) || (sensitivity == NULL) ||
      (config_valid(&device->config) == 0u)) {
    return APP_STATUS_BAD_PARAM;
  }

  /* HALLCONF=0xC 和 0x0 必须使用不同表，不能只按增益和分辨率推导。 */
  table = (device->config.hallconf == 0x0Cu) ? g_sensitivity_hallconf_c
                                             : g_sensitivity_hallconf_0;
  gain = device->config.gain_sel;
  sensitivity->x_ut_per_lsb = table[gain][device->config.res_x].xy_ut_per_lsb;
  sensitivity->y_ut_per_lsb = table[gain][device->config.res_y].xy_ut_per_lsb;
  sensitivity->z_ut_per_lsb = table[gain][device->config.res_z].z_ut_per_lsb;
  return APP_STATUS_OK;
}

/** @brief 发送只返回状态字节的 MLX 命令并检查 ERROR 位。 */
static AppStatus command_status(Mlx90393 *device, uint8_t command) {
  uint8_t status;
  AppStatus result = BspI2c_CommandRead(device->address, command, &status, 1u);
  if (result != APP_STATUS_OK) return result;
  device->last_status = status;
  return ((status & MLX_STATUS_ERROR) == 0u) ? APP_STATUS_OK : APP_STATUS_IO;
}

/** @brief 实现 RR 命令及 16 位大端寄存器数据还原。 */
AppStatus Mlx90393_ReadRegister(Mlx90393 *device, uint8_t reg, uint16_t *value) {
  uint8_t rx[3];
  AppStatus result;
  if ((device == NULL) || (value == NULL) || (reg > 0x3Fu)) return APP_STATUS_BAD_PARAM;
  result = BspI2c_CommandAddressRead(device->address, MLX_CMD_READ_REGISTER,
                                     (uint8_t)(reg << 2), rx, sizeof(rx));
  if (result != APP_STATUS_OK) return result;
  device->last_status = rx[0];
  if ((rx[0] & MLX_STATUS_ERROR) != 0u) return APP_STATUS_IO;
  *value = (uint16_t)(((uint16_t)rx[1] << 8) | rx[2]);
  return APP_STATUS_OK;
}

/** @brief 实现 WR 命令以及 repeated START 状态读取。 */
AppStatus Mlx90393_WriteRegister(Mlx90393 *device, uint8_t reg, uint16_t value) {
  uint8_t tx[4] = {MLX_CMD_WRITE_REGISTER, (uint8_t)(value >> 8),
                   (uint8_t)value, (uint8_t)(reg << 2)};
  uint8_t status;
  AppStatus result;
  if ((device == NULL) || (reg > 0x3Fu)) return APP_STATUS_BAD_PARAM;
  result = BspI2c_CommandWriteReadStatus(device->address, tx, sizeof(tx), &status);
  if (result != APP_STATUS_OK) return result;
  device->last_status = status;
  return ((status & MLX_STATUS_ERROR) == 0u) ? APP_STATUS_OK : APP_STATUS_IO;
}

/**
 * @brief 实现 EX、RT、寄存器配置和回读校验的完整初始化流程。
 * @details 按数据手册寄存器图，Reg0 高 8 位 ANA_RESERVED_LOW/BIST 保留原值，低 8 位写入
 *          Z_SERIES/GAIN_SEL/HALLCONF；Reg1 仅关闭 TCMP_EN；Reg2 使用项目采样配置。
 */
AppStatus Mlx90393_Init(Mlx90393 *device, uint8_t address,
                        const Mlx90393Config *config) {
  AppStatus result;
  uint16_t reg0;
  uint16_t reg1;
  uint16_t reg2;
  uint16_t verify;
  if ((device == NULL) || (config_valid(config) == 0u)) return APP_STATUS_BAD_PARAM;
  device->address = address;
  device->initialized = 0u;
  device->last_init_step = 1u;
  device->last_status = 0u;
  device->config = *config;
  /* 第一步退出可能仍在运行的 Burst/WOC/SM 模式，再执行软复位。 */
  result = command_status(device, MLX_CMD_EXIT);
  if (result != APP_STATUS_OK) return result;
  BspTime_DelayMs(2u);
  /* RT returns a status byte; keep the same command+status transaction as the reference driver. */
  device->last_init_step = 2u;
  result = command_status(device, MLX_CMD_RESET);
  if (result != APP_STATUS_OK) return result;
  BspTime_DelayMs(2u);
  /* 读取现有寄存器，以便保留芯片保留位而非整字盲写。 */
  device->last_init_step = 3u;
  result = Mlx90393_ReadRegister(device, 0u, &reg0);
  if (result != APP_STATUS_OK) return result;
  device->last_init_step = 4u;
  result = Mlx90393_ReadRegister(device, 1u, &reg1);
  if (result != APP_STATUS_OK) return result;
  /* Reg0 高字节保留原值；低字节写入增益和 Hall spinning 配置。 */
  reg0 = (uint16_t)((reg0 & 0xFF00u) | build_reg0_config(config));
  reg1 = build_reg1_config(reg1, config);
  reg2 = build_reg2_config(config);
  /* 按 Reg0、Reg1、Reg2 顺序写入易失配置，任一步失败立即退出。 */
  device->last_init_step = 5u;
  result = Mlx90393_WriteRegister(device, 0u, reg0);
  if (result != APP_STATUS_OK) return result;
  device->last_init_step = 6u;
  result = Mlx90393_WriteRegister(device, 1u, reg1);
  if (result != APP_STATUS_OK) return result;
  device->last_init_step = 7u;
  result = Mlx90393_WriteRegister(device, 2u, reg2);
  if (result != APP_STATUS_OK) return result;
  /* 写后逐项回读，区分通信错误与寄存器值不一致。 */
  device->last_init_step = 8u;
  result = Mlx90393_ReadRegister(device, 0u, &verify);
  if (result != APP_STATUS_OK) return result;
  if (verify != reg0) return APP_STATUS_VERIFY;
  device->last_init_step = 9u;
  result = Mlx90393_ReadRegister(device, 1u, &verify);
  if (result != APP_STATUS_OK) return result;
  if (verify != reg1) return APP_STATUS_VERIFY;
  device->last_init_step = 10u;
  result = Mlx90393_ReadRegister(device, 2u, &verify);
  if (result != APP_STATUS_OK) return result;
  if (verify != reg2) return APP_STATUS_VERIFY;
  device->initialized = 1u;
  device->last_init_step = 0u;
  return APP_STATUS_OK;
}

/** @brief 实现 SM XYZT 命令。 */
AppStatus Mlx90393_StartSingle(Mlx90393 *device) {
  if ((device == NULL) || (device->initialized == 0u)) return APP_STATUS_NOT_READY;
  return command_status(device, MLX_CMD_START_XYZT);
}

/** @brief 实现 SB zyxt 命令，默认 zyxt=0xF 时连续转换温度和 XYZ。 */
AppStatus Mlx90393_StartBurst(Mlx90393 *device) {
  uint8_t status;
  uint8_t burst_sel;
  AppStatus result;
  if ((device == NULL) || (device->initialized == 0u)) return APP_STATUS_NOT_READY;
  burst_sel = (device->config.burst_sel != 0u)
                  ? (uint8_t)(device->config.burst_sel & MLX_CMD_XYZT_MASK)
                  : MLX_CMD_XYZT_MASK;
  result = BspI2c_CommandRead(device->address,
                              (uint8_t)(MLX_CMD_START_BURST_BASE | burst_sel),
                              &status, 1u);
  if (result != APP_STATUS_OK) return result;
  device->last_status = status;
  if ((status & MLX_STATUS_ERROR) != 0u) return APP_STATUS_IO;
  return ((status & MLX_STATUS_BURST_MODE) != 0u) ? APP_STATUS_OK : APP_STATUS_IO;
}

/** @brief 实现 RM XYZT 命令、状态检查和原始值还原。 */
AppStatus Mlx90393_Read(Mlx90393 *device, Mlx90393Sample *sample) {
  uint8_t rx[9];
  uint8_t command = MLX_CMD_READ_XYZT;
  AppStatus result;
  if ((device == NULL) || (sample == NULL)) return APP_STATUS_BAD_PARAM;
  if (device->initialized == 0u) return APP_STATUS_NOT_READY;
  /* RM 返回 Status-T-X-Y-Z，共九字节，所有 16 位量均为高字节在前。 */
#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND != 0u)
  result = BspI2c_CommandReadTimeout(device->address, command, rx, sizeof(rx),
                                     BOARD_I2C_RUNTIME_TIMEOUT_MS);
#else
  result = BspI2c_CommandRead(device->address, command, rx, sizeof(rx));
#endif
  if (result != APP_STATUS_OK) return result;
  sample->status = rx[0];
  /* MLX temperature is unsigned; 46244 is the 25 C zero point. */
  sample->temperature = (int16_t)((int32_t)(((uint16_t)rx[1] << 8) | rx[2]) -
                                  (int32_t)MLX_TEMP_ZERO_RAW);
  sample->x = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
  sample->y = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  sample->z = (int16_t)(((uint16_t)rx[7] << 8) | rx[8]);
  /* Keep the received XYZT for diagnostics even when the status is abnormal. */
  if ((rx[0] & MLX_STATUS_ERROR) != 0u || rm_response_invalid(rx) != 0u) {
    return APP_STATUS_IO;
  }
  return APP_STATUS_OK;
}












