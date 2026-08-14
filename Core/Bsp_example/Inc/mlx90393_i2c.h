#ifndef MLX90393_I2C_H
#define MLX90393_I2C_H

#include <stdint.h>
#include "app_config.h"
#include "app_types.h"

/* 传感器一次 RM 读回来的原始寄存器值。 */
typedef struct {
  uint16_t t;
  uint16_t x;
  uint16_t y;
  uint16_t z;
} MlxRawData;

/* 把原始值按当前增益/分辨率配置换算后的工程量。 */
typedef struct {
  float temp_c;
  float mag_x_ut;
  float mag_y_ut;
  float mag_z_ut;
} MlxParsedData;

/* 每颗传感器的累计通信统计。 */
typedef struct {
  uint32_t ok_cnt;
  uint32_t nack_cnt;
  uint32_t status_err_cnt;
  uint32_t timeout_cnt;
} SensorStats;

/* 上电初始化详细报告：
 * 便于定位 EX/RT/寄存器写入/回读校验究竟卡在哪一步。
 */
typedef struct {
  AppRet ex_ret;
  AppRet rt_ret;
  AppRet reg0_read_ret;
  AppRet reg0_write_ret;
  AppRet reg1_write_ret;
  AppRet reg2_write_ret;
  AppRet reg0_verify_ret;
  AppRet reg1_verify_ret;
  AppRet reg2_verify_ret;
  uint8_t ex_status;
  uint8_t rt_status;
  uint8_t factory_reg0_low8;
  uint16_t reg0_expected;
  uint16_t reg1_expected;
  uint16_t reg2_expected;
  uint16_t reg0_readback;
  uint16_t reg1_readback;
  uint16_t reg2_readback;
} MlxInitReport;

/* 单颗初始化；成功后该颗会进入项目当前配置定义的运行模式。 */
AppRet mlx90393_init_one(uint8_t bus_id, uint8_t addr7);
/* 带详细诊断信息的初始化版本，供 boot log 和排障使用。 */
AppRet mlx90393_init_one_with_report(uint8_t bus_id, uint8_t addr7, MlxInitReport *report);
/* 发送单次测量命令 SM。 */
AppRet mlx90393_sm(uint8_t bus_id, uint8_t addr7, uint8_t *status);
/* 多条总线上、相同从地址的锁步 SM。 */
void mlx90393_sm_lockstep_same_addr(uint16_t bus_mask, uint8_t addr7, uint8_t *status_by_bus,
                                    AppRet *ret_by_bus);
/* 读取单颗最新测量结果 RM。 */
AppRet mlx90393_read_meas(uint8_t bus_id, uint8_t addr7, MlxRawData *out);
/* 多条总线上、相同从地址的锁步 RM。 */
void mlx90393_read_meas_lockstep_same_addr(uint16_t bus_mask, uint8_t addr7, MlxRawData *raw_by_bus,
                                           AppRet *ret_by_bus);
/* 将 RM 原始值换算成摄氏度和 uT。 */
AppRet mlx90393_parse(const MlxRawData *raw, MlxParsedData *out);
/* 寄存器读写接口，主要给初始化流程使用。 */
AppRet mlx90393_write_reg(uint8_t bus_id, uint8_t addr7, uint8_t reg, uint16_t val);
AppRet mlx90393_read_reg(uint8_t bus_id, uint8_t addr7, uint8_t reg, uint16_t *val);

#endif /* MLX90393_I2C_H */
