#include "mlx90393_i2c.h"

#include <stdbool.h>
#include "app_config.h"
#include "soft_i2c.h"

/* MLX90393 驱动层：
 * 这里封装了寄存器初始化、单次测量 SM、结果读取 RM，以及原始值到工程量的换算。
 * 上层 collector 不直接关心寄存器细节，只通过这里访问传感器。
 */

#define MLX_CMD_EX 0x80u
#define MLX_CMD_RT 0xF0u
#define MLX_CMD_SM 0x3Fu
#define MLX_CMD_RM 0x4Fu
#define MLX_CMD_WR 0x60u
#define MLX_CMD_RR 0x50u
#define MLX_STATUS_ERR_MASK 0x10u

#define REG0_GAIN_SEL_SHIFT 12u
#define REG0_HALLCONF_SHIFT 8u

#define REG1_TCMP_EN_SHIFT 10u

#define REG2_OSR_SHIFT 0u
#define REG2_DIG_FILT_SHIFT 2u
#define REG2_RES_X_SHIFT 5u
#define REG2_RES_Y_SHIFT 7u
#define REG2_RES_Z_SHIFT 9u
#define REG2_OSR2_SHIFT 11u

/* 项目运行时使用的 MLX 配置参数集合。 */
typedef struct {
  uint8_t gain_sel;
  uint8_t hallconf;
  uint8_t res_x;
  uint8_t res_y;
  uint8_t res_z;
  uint8_t dig_filt;
  uint8_t osr;
  uint8_t osr2;
} mlx_runtime_cfg_t;

/* 灵敏度表中的单个系数项。 */
typedef struct {
  float xy_ut_per_lsb;
  float z_ut_per_lsb;
} mlx_sens_t;

/* 编译期目标配置。 */
static const mlx_runtime_cfg_t g_mlx_cfg = {
    .gain_sel = APP_CFG_MLX_GAIN_SEL,
    .hallconf = APP_CFG_MLX_HALLCONF,
    .res_x = APP_CFG_MLX_RES_X,
    .res_y = APP_CFG_MLX_RES_Y,
    .res_z = APP_CFG_MLX_RES_Z,
    .dig_filt = APP_CFG_MLX_DIG_FILT,
    .osr = APP_CFG_MLX_OSR,
    .osr2 = APP_CFG_MLX_OSR2,
};

/* HallConf=0xC 和 HallConf=0x0 时的灵敏度查找表。 */
static const mlx_sens_t g_sens_table_hallconf_c[8][4] = {
    {{0.751f, 1.210f}, {1.502f, 2.420f}, {3.004f, 4.840f}, {6.009f, 9.680f}},
    {{0.601f, 0.968f}, {1.202f, 1.936f}, {2.403f, 3.872f}, {4.840f, 7.744f}},
    {{0.451f, 0.726f}, {0.901f, 1.452f}, {1.803f, 2.904f}, {3.605f, 5.808f}},
    {{0.376f, 0.605f}, {0.751f, 1.210f}, {1.502f, 2.420f}, {3.004f, 4.840f}},
    {{0.300f, 0.484f}, {0.601f, 0.968f}, {1.202f, 1.936f}, {2.403f, 3.872f}},
    {{0.250f, 0.403f}, {0.501f, 0.807f}, {1.001f, 1.613f}, {2.003f, 3.227f}},
    {{0.200f, 0.323f}, {0.401f, 0.645f}, {0.801f, 1.291f}, {1.602f, 2.581f}},
    {{0.150f, 0.242f}, {0.300f, 0.484f}, {0.601f, 0.968f}, {1.202f, 1.936f}},
};

static const mlx_sens_t g_sens_table_hallconf_0[8][4] = {
    {{0.787f, 1.267f}, {1.573f, 2.534f}, {3.146f, 5.068f}, {6.292f, 10.137f}},
    {{0.629f, 1.014f}, {1.258f, 2.027f}, {2.517f, 4.055f}, {5.034f, 8.109f}},
    {{0.472f, 0.760f}, {0.944f, 1.521f}, {1.888f, 3.041f}, {3.775f, 6.082f}},
    {{0.393f, 0.634f}, {0.787f, 1.267f}, {1.573f, 2.534f}, {3.146f, 5.068f}},
    {{0.315f, 0.507f}, {0.629f, 1.014f}, {1.258f, 2.027f}, {2.517f, 4.055f}},
    {{0.262f, 0.422f}, {0.524f, 0.845f}, {1.049f, 1.689f}, {2.097f, 3.379f}},
    {{0.210f, 0.338f}, {0.419f, 0.676f}, {0.839f, 1.352f}, {1.678f, 2.703f}},
    {{0.157f, 0.253f}, {0.315f, 0.507f}, {0.629f, 1.014f}, {1.258f, 2.027f}},
};

static uint16_t g_mlx_regs[3] = {0u, APP_MLX_REG1_CFG, 0u};
static mlx_runtime_cfg_t g_mlx_effective_cfg = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
static uint8_t g_mlx_tcmp_en = 0u;

/* 判断一段 MLX 读回是否全部为 0xFF。
 * 全 1 通常表示 SDA 在整个读窗口都没有被从机拉低；它可能来自边沿过快、线路上拉/电容不匹配
 * 或器件尚未正确响应。状态字节 0xFF 的 bit0 仍为 1，因此只检查 bit0 会把这种总线假读当成成功。 */
static uint8_t mlx90393_rx_all_ff(const uint8_t *rx, uint8_t len) {
  if ((rx == NULL) || (len == 0u)) {
    return 0u;
  }
  for (uint8_t i = 0u; i < len; i++) {
    if (rx[i] != 0xFFu) {
      return 0u;
    }
  }
  return 1u;
}

static uint8_t mlx90393_xyz_all_ffff(const uint8_t *rx) {
  return ((rx != NULL) &&
          (rx[3] == 0xFFu) && (rx[4] == 0xFFu) &&
          (rx[5] == 0xFFu) && (rx[6] == 0xFFu) &&
          (rx[7] == 0xFFu) && (rx[8] == 0xFFu)) ? 1u : 0u;
}

static float g_scale_x_ut_per_lsb = 0.0f;
static float g_scale_y_ut_per_lsb = 0.0f;
static float g_scale_z_ut_per_lsb = 0.0f;
static bool g_mlx_cfg_ready = false;

/* 初始化报告默认值，保证失败时未填字段也有可读结果。 */
static void fill_init_report_defaults(MlxInitReport *report) {
  if (report == NULL) {
    return;
  }
  report->ex_ret = APP_ERR_PARAM;
  report->rt_ret = APP_ERR_PARAM;
  report->reg0_read_ret = APP_ERR_PARAM;
  report->reg0_write_ret = APP_ERR_PARAM;
  report->reg1_write_ret = APP_ERR_PARAM;
  report->reg2_write_ret = APP_ERR_PARAM;
  report->reg0_verify_ret = APP_ERR_PARAM;
  report->reg1_verify_ret = APP_ERR_PARAM;
  report->reg2_verify_ret = APP_ERR_PARAM;
  report->ex_status = 0u;
  report->rt_status = 0u;
  report->factory_reg0_low8 = 0u;
  report->reg0_expected = 0u;
  report->reg1_expected = 0u;
  report->reg2_expected = 0u;
  report->reg0_readback = 0u;
  report->reg1_readback = 0u;
  report->reg2_readback = 0u;
}

/* 依据当前配置查出 XYZ 三轴的换算系数。 */
static void get_axis_scale_from_cfg(const mlx_runtime_cfg_t *cfg, float *kx, float *ky, float *kz) {
  const mlx_sens_t(*tbl)[4] = g_sens_table_hallconf_c;
  if ((cfg->hallconf & 0x0Fu) == 0x00u) {
    /* HallConf=0 与项目默认 HallConf=0xC 需要查不同灵敏度表。 */
    tbl = g_sens_table_hallconf_0;
  }

  *kx = tbl[cfg->gain_sel & 0x07u][cfg->res_x & 0x03u].xy_ut_per_lsb;
  *ky = tbl[cfg->gain_sel & 0x07u][cfg->res_y & 0x03u].xy_ut_per_lsb;
  *kz = tbl[cfg->gain_sel & 0x07u][cfg->res_z & 0x03u].z_ut_per_lsb;
}

/* 根据项目配置生成目标寄存器值。 */
static uint16_t build_reg0_high8(const mlx_runtime_cfg_t *cfg) {
  return (uint16_t)(((cfg->gain_sel & 0x07u) << REG0_GAIN_SEL_SHIFT) |
                    ((cfg->hallconf & 0x0Fu) << REG0_HALLCONF_SHIFT));
}

static uint16_t build_reg2(const mlx_runtime_cfg_t *cfg) {
  return (uint16_t)(((cfg->res_y & 0x03u) << REG2_RES_Y_SHIFT) |
                    ((cfg->res_x & 0x03u) << REG2_RES_X_SHIFT) |
                    ((cfg->dig_filt & 0x07u) << REG2_DIG_FILT_SHIFT) |
                    ((cfg->osr & 0x03u) << REG2_OSR_SHIFT) |
                    ((cfg->osr2 & 0x03u) << REG2_OSR2_SHIFT) |
                    ((cfg->res_z & 0x03u) << REG2_RES_Z_SHIFT));
}

/* 把编译期配置刷新到运行寄存器镜像。 */
static void refresh_reg_config_from_cfg(void) {
  g_mlx_regs[0] = build_reg0_high8(&g_mlx_cfg);
  g_mlx_regs[1] = APP_MLX_REG1_CFG;
  g_mlx_regs[2] = build_reg2(&g_mlx_cfg);
}

/* 以编译期配置为准初始化当前有效换算参数。 */
static void set_effective_cfg_default(void) {
  g_mlx_effective_cfg = g_mlx_cfg;
  g_mlx_tcmp_en = (uint8_t)((g_mlx_regs[1] >> REG1_TCMP_EN_SHIFT) & 0x01u);
  get_axis_scale_from_cfg(&g_mlx_effective_cfg, &g_scale_x_ut_per_lsb, &g_scale_y_ut_per_lsb,
                          &g_scale_z_ut_per_lsb);
}

/* 以实际回读寄存器为准初始化当前有效换算参数。 */
static void set_effective_cfg_from_regs(uint16_t reg0, uint16_t reg1, uint16_t reg2) {
  g_mlx_effective_cfg = g_mlx_cfg;
  g_mlx_effective_cfg.gain_sel = (uint8_t)((reg0 >> REG0_GAIN_SEL_SHIFT) & 0x07u);
  g_mlx_effective_cfg.hallconf = (uint8_t)((reg0 >> REG0_HALLCONF_SHIFT) & 0x0Fu);
  g_mlx_effective_cfg.res_x = (uint8_t)((reg2 >> REG2_RES_X_SHIFT) & 0x03u);
  g_mlx_effective_cfg.res_y = (uint8_t)((reg2 >> REG2_RES_Y_SHIFT) & 0x03u);
  g_mlx_effective_cfg.res_z = (uint8_t)((reg2 >> REG2_RES_Z_SHIFT) & 0x03u);
  g_mlx_effective_cfg.dig_filt = (uint8_t)((reg2 >> REG2_DIG_FILT_SHIFT) & 0x07u);
  g_mlx_effective_cfg.osr = (uint8_t)((reg2 >> REG2_OSR_SHIFT) & 0x03u);
  g_mlx_effective_cfg.osr2 = (uint8_t)((reg2 >> REG2_OSR2_SHIFT) & 0x03u);
  g_mlx_tcmp_en = (uint8_t)((reg1 >> REG1_TCMP_EN_SHIFT) & 0x01u);

  get_axis_scale_from_cfg(&g_mlx_effective_cfg, &g_scale_x_ut_per_lsb, &g_scale_y_ut_per_lsb,
                          &g_scale_z_ut_per_lsb);
}

/* 惰性初始化寄存器镜像和换算参数。 */
static void ensure_mlx_cfg_ready(void) {
  if (!g_mlx_cfg_ready) {
    refresh_reg_config_from_cfg();
    set_effective_cfg_default();
    g_mlx_cfg_ready = true;
  }
}

/* 按手册 Table 21 规则，把原始无符号输出还原成有符号物理值。 */
static int32_t decode_axis_raw_table21(uint16_t raw, uint8_t res, uint8_t tcmp_en) {
  uint8_t res_sel = (uint8_t)(res & 0x03u);

  if ((tcmp_en & 0x01u) == 0u) {
    /* TCMP 关闭时，不同分辨率对应不同的零点偏移规则。 */
    switch (res_sel) {
      case 0u:
      case 1u:
        return (int32_t)(int16_t)raw;
      case 2u:
        return (int32_t)raw - (1 << 15);
      case 3u:
      default:
        return (int32_t)raw - (1 << 14);
    }
  }

  if (res_sel <= 1u) {
    /* TCMP 打开后，低分辨率分支按另一套偏移规则解码。 */
    return (int32_t)raw - (1 << 15);
  }

  /* 当前项目不使用 TCMP+高分辨率组合，这里保守返回 0。 */
  return 0;
}

/* 发送单字节命令并读回状态字。 */
static AppRet mlx_cmd_with_status(uint8_t bus_id, uint8_t addr7, uint8_t cmd, uint8_t *status) {
  uint8_t rx[1];
  AppRet ret;
  if (status == 0) {
    return APP_ERR_PARAM;
  }

  ret = soft_i2c_write_read_u8(bus_id, addr7, cmd, rx, 1u);
  if (ret != APP_OK) {
    return ret;
  }
  *status = rx[0];
  return APP_OK;
}

/* 寄存器写接口。 */
AppRet mlx90393_write_reg(uint8_t bus_id, uint8_t addr7, uint8_t reg, uint16_t val) {
  uint8_t tx[4];
  uint8_t rx[1];
  AppRet ret;
  tx[0] = MLX_CMD_WR;
  tx[1] = (uint8_t)((val >> 8) & 0xFFu);
  tx[2] = (uint8_t)(val & 0xFFu);
  tx[3] = (uint8_t)((reg << 2) & 0xFCu);
  ret = soft_i2c_write_read(bus_id, addr7, tx, 4u, rx, 1u);
  if (ret != APP_OK) {
    return ret;
  }
  /* MLX 命令层即便 I2C 成功，也仍可能通过状态位报告设备级错误。 */
  if ((rx[0] & MLX_STATUS_ERR_MASK) != 0u) {
    return APP_ERR_MLX_STATUS;
  }
  return APP_OK;
}

/* 寄存器读接口。 */
AppRet mlx90393_read_reg(uint8_t bus_id, uint8_t addr7, uint8_t reg, uint16_t *val) {
  uint8_t tx[2];
  uint8_t rx[3];
  AppRet ret;
  if (val == 0) {
    return APP_ERR_PARAM;
  }
  tx[0] = MLX_CMD_RR;
  tx[1] = (uint8_t)((reg << 2) & 0xFCu);
  ret = soft_i2c_write_read(bus_id, addr7, tx, 2u, rx, 3u);
  if (ret != APP_OK) {
    return ret;
  }
  /* RR 成功后返回的是“状态字 + 16-bit 寄存器值”。 */
  if ((rx[0] & MLX_STATUS_ERR_MASK) != 0u) {
    return APP_ERR_MLX_STATUS;
  }
  *val = (uint16_t)(((uint16_t)rx[1] << 8) | rx[2]);
  return APP_OK;
}

/* 单颗初始化完整流程：
 * EX -> RT -> 读取并保留 Reg0 低 8 位校准 -> 写 Reg0/1/2 -> 回读校验 -> 更新有效换算参数。
 */
AppRet mlx90393_init_one_with_report(uint8_t bus_id, uint8_t addr7, MlxInitReport *report) {
  uint8_t status = 0u;
  AppRet ret;
  uint16_t expected_reg0 = 0u;
  uint16_t reg0_readback = 0u;
  uint16_t reg1_readback = 0u;
  uint16_t reg2_readback = 0u;
  uint16_t reg_chk = 0u;

  fill_init_report_defaults(report);
  ensure_mlx_cfg_ready();
  set_effective_cfg_default();
  expected_reg0 = g_mlx_regs[0];
  if (report != NULL) {
    /* 先把理论目标值写进 report，后面如果为了保留校准值发生调整，再覆盖更新。 */
    report->reg0_expected = expected_reg0;
    report->reg1_expected = g_mlx_regs[1];
    report->reg2_expected = g_mlx_regs[2];
  }

  /* Step 1: EX + RT soft reset */
  ret = mlx_cmd_with_status(bus_id, addr7, MLX_CMD_EX, &status);
  if (report != NULL) {
    report->ex_ret = ret;
    report->ex_status = status;
  }
  if (ret != APP_OK) {
    return ret;
  }
#if (APP_MLX_EX_RT_DELAY_MS > 0U)
  /* EX 后必须给芯片一点时间完成退出/复位动作。 */
  HAL_Delay(APP_MLX_EX_RT_DELAY_MS);
#else
  HAL_Delay(1u);
#endif

  ret = mlx_cmd_with_status(bus_id, addr7, MLX_CMD_RT, &status);
  if (report != NULL) {
    report->rt_ret = ret;
    report->rt_status = status;
  }
  if (ret != APP_OK) {
    return ret;
  }
  /* 手册 RT 后建议约 1.5ms；见 app_config APP_MLX_RT_POST_DELAY_MS */
#if (APP_MLX_RT_POST_DELAY_MS > 0U)
  HAL_Delay(APP_MLX_RT_POST_DELAY_MS);
#endif

  /* Step 2: 安全写 Reg0，保留出厂低 8 位校准值 */
  ret = mlx90393_read_reg(bus_id, addr7, 0u, &reg_chk);
  if (report != NULL) {
    report->reg0_read_ret = ret;
    report->reg0_readback = reg_chk;
  }
  if (ret == APP_OK) {
    /* Reg0 低 8 位是出厂校准位，必须保留下来，不能直接用项目配置整字覆盖。 */
    expected_reg0 = (uint16_t)((g_mlx_regs[0] & 0xFF00u) | (reg_chk & 0x00FFu));
    if (report != NULL) {
      report->factory_reg0_low8 = (uint8_t)(reg_chk & 0x00FFu);
    }
  } else if (report != NULL) {
    report->factory_reg0_low8 = 0u;
  }
  if (report != NULL) {
    report->reg0_expected = expected_reg0;
  }
  ret = mlx90393_write_reg(bus_id, addr7, 0u, expected_reg0);
  if (report != NULL) {
    report->reg0_write_ret = ret;
  }
  if (ret != APP_OK) {
    return ret;
  }

  /* Step 3: 写 Reg1 */
  ret = mlx90393_write_reg(bus_id, addr7, 1u, g_mlx_regs[1]);
  if (report != NULL) {
    report->reg1_write_ret = ret;
  }
  if (ret != APP_OK) {
    return ret;
  }

  /* Step 4: 写 Reg2 */
  ret = mlx90393_write_reg(bus_id, addr7, 2u, g_mlx_regs[2]);
  if (report != NULL) {
    report->reg2_write_ret = ret;
  }
  if (ret != APP_OK) {
    return ret;
  }

  /* Step 5: 回读校验 Reg0/Reg1/Reg2 */
  ret = mlx90393_read_reg(bus_id, addr7, 0u, &reg0_readback);
  if (report != NULL) {
    report->reg0_verify_ret = ret;
    report->reg0_readback = reg0_readback;
  }
  if (ret != APP_OK) {
    return ret;
  }
  if (reg0_readback != expected_reg0) {
    return APP_ERR_MLX_STATUS;
  }

  ret = mlx90393_read_reg(bus_id, addr7, 1u, &reg1_readback);
  if (report != NULL) {
    report->reg1_verify_ret = ret;
    report->reg1_readback = reg1_readback;
  }
  if (ret != APP_OK) {
    return ret;
  }
  if (reg1_readback != g_mlx_regs[1]) {
    return APP_ERR_MLX_STATUS;
  }

  ret = mlx90393_read_reg(bus_id, addr7, 2u, &reg2_readback);
  if (report != NULL) {
    report->reg2_verify_ret = ret;
    report->reg2_readback = reg2_readback;
  }
  if (ret != APP_OK) {
    return ret;
  }
  if (reg2_readback != g_mlx_regs[2]) {
    /* 回读不匹配时统一按状态错误处理，避免“貌似初始化成功，实际参数没生效”。 */
    return APP_ERR_MLX_STATUS;
  }

  set_effective_cfg_from_regs(reg0_readback, reg1_readback, reg2_readback);
#if (APP_MLX_INIT_DONE_DELAY_MS > 0U)
  HAL_Delay(APP_MLX_INIT_DONE_DELAY_MS);
#endif
  return APP_OK;
}

AppRet mlx90393_init_one(uint8_t bus_id, uint8_t addr7) {
  return mlx90393_init_one_with_report(bus_id, addr7, NULL);
}

AppRet mlx90393_sm(uint8_t bus_id, uint8_t addr7, uint8_t *status) {
  AppRet ret = mlx_cmd_with_status(bus_id, addr7, MLX_CMD_SM, status);
  if (ret != APP_OK) {
    return ret;
  }
  /* 当前上层只要求通信成功；状态字的具体含义由 collector 另行保存。 */
  return APP_OK;
}

void mlx90393_sm_lockstep_same_addr(uint16_t bus_mask, uint8_t addr7, uint8_t *status_by_bus,
                                    AppRet *ret_by_bus) {
  uint8_t tx = MLX_CMD_SM;
  uint8_t rx_by_bus[SENSOR_BUS_COUNT];

  if ((status_by_bus == 0) || (ret_by_bus == 0)) {
    return;
  }

  soft_i2c_write_read_lockstep_same_addr(bus_mask, addr7, &tx, 1u, rx_by_bus, 1u, ret_by_bus);
  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    /* status_by_bus 固定按 bus_id 索引，便于上层直接查。 */
    status_by_bus[bus_id] = rx_by_bus[bus_id];
    /* 0xFF 表示整个状态字节的 8 个 bit 都读成高电平。MLX 状态的 bit0 恰好也会是 1，
     * 如果只沿用“bit0=1 即成功”的判断，总线未被器件驱动、边沿过快或读时序失配时，
     * 这类假读会被误判为有效 SM。把它转成状态错误后，collector 会清除本帧 valid、
     * 累计 miss，并在连续失败达到阈值时调用现有的 9-SCL 总线恢复。 */
    if (((bus_mask & (uint16_t)(1u << bus_id)) != 0u) &&
        (ret_by_bus[bus_id] == APP_OK) &&
        (rx_by_bus[bus_id] == 0xFFu)) {
      ret_by_bus[bus_id] = APP_ERR_MLX_STATUS;
    }
  }
}

AppRet mlx90393_read_meas(uint8_t bus_id, uint8_t addr7, MlxRawData *out) {
  uint8_t rx[9];
  uint8_t cmd = MLX_CMD_RM;
  AppRet ret;
  if (out == 0) {
    return APP_ERR_PARAM;
  }

  ret = soft_i2c_write_read_u8(bus_id, addr7, cmd, rx, 9u);
  if (ret != APP_OK) {
    return ret;
  }

  /* RM 返回的第 1 个字节仍是状态字，不是温度高字节。 */
  if ((mlx90393_rx_all_ff(rx, (uint8_t)sizeof(rx)) != 0u) ||
      (mlx90393_xyz_all_ffff(rx) != 0u) ||
      ((rx[0] & 0x01u) == 0u)) {
    return APP_ERR_MLX_STATUS;
  }

  out->t = (uint16_t)(((uint16_t)rx[1] << 8) | rx[2]);
  out->x = (uint16_t)(((uint16_t)rx[3] << 8) | rx[4]);
  out->y = (uint16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  out->z = (uint16_t)(((uint16_t)rx[7] << 8) | rx[8]);
  return APP_OK;
}

void mlx90393_read_meas_lockstep_same_addr(uint16_t bus_mask, uint8_t addr7, MlxRawData *raw_by_bus,
                                           AppRet *ret_by_bus) {
  uint8_t cmd = MLX_CMD_RM;
  uint8_t rx_by_bus[SENSOR_BUS_COUNT * 9u];

  if ((raw_by_bus == 0) || (ret_by_bus == 0)) {
    return;
  }

  soft_i2c_write_read_lockstep_same_addr(bus_mask, addr7, &cmd, 1u, rx_by_bus, 9u, ret_by_bus);
  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    uint8_t *rx = &rx_by_bus[bus_id * 9u];

    if ((bus_mask & (uint16_t)(1u << bus_id)) == 0u) {
      /* 非目标总线直接跳过，保留调用方原缓冲内容。 */
      continue;
    }
    if (ret_by_bus[bus_id] != APP_OK) {
      /* I2C 层已经报错的总线，不再继续解析 payload。 */
      continue;
    }
    /* 拒绝全 0xFF 后再检查 MLX 状态位，避免输出 valid=1、四通道均为 65535 的假样本。 */
    if ((mlx90393_rx_all_ff(rx, 9u) != 0u) ||
        (mlx90393_xyz_all_ffff(rx) != 0u) ||
        ((rx[0] & 0x01u) == 0u)) {
      ret_by_bus[bus_id] = APP_ERR_MLX_STATUS;
      continue;
    }

    raw_by_bus[bus_id].t = (uint16_t)(((uint16_t)rx[1] << 8) | rx[2]);
    raw_by_bus[bus_id].x = (uint16_t)(((uint16_t)rx[3] << 8) | rx[4]);
    raw_by_bus[bus_id].y = (uint16_t)(((uint16_t)rx[5] << 8) | rx[6]);
    raw_by_bus[bus_id].z = (uint16_t)(((uint16_t)rx[7] << 8) | rx[8]);
  }
}

AppRet mlx90393_parse(const MlxRawData *raw, MlxParsedData *out) {
  int32_t x_signed;
  int32_t y_signed;
  int32_t z_signed;
  if ((raw == 0) || (out == 0)) {
    return APP_ERR_PARAM;
  }

  ensure_mlx_cfg_ready();
  /* 先把三个轴从设备编码恢复成有符号原始值，再乘灵敏度得到 uT。 */
  x_signed = decode_axis_raw_table21(raw->x, g_mlx_effective_cfg.res_x, g_mlx_tcmp_en);
  y_signed = decode_axis_raw_table21(raw->y, g_mlx_effective_cfg.res_y, g_mlx_tcmp_en);
  z_signed = decode_axis_raw_table21(raw->z, g_mlx_effective_cfg.res_z, g_mlx_tcmp_en);

  out->temp_c = 25.0f + ((float)raw->t - APP_MLX_TEMP_OFFSET) / APP_MLX_TEMP_LSB_PER_C;
  out->mag_x_ut = (float)x_signed * g_scale_x_ut_per_lsb;
  out->mag_y_ut = (float)y_signed * g_scale_y_ut_per_lsb;
  out->mag_z_ut = (float)z_signed * g_scale_z_ut_per_lsb;
  return APP_OK;
}
