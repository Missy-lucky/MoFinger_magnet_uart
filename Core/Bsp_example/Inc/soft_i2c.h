#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdbool.h>
#include <stdint.h>
#include "app_types.h"
#include "sensor_topology.h"

/* 软 I2C 每条总线的运行上下文。 */
typedef struct {
  SoftI2CBusPins pins;
  uint16_t delay_us;
  uint16_t ack_timeout_us;
} SoftI2CBusCtx;

/* 初始化全部软 I2C 总线。 */
AppRet soft_i2c_init_all(const SoftI2CBusPins *pins, uint8_t bus_count, uint16_t delay_us,
                         uint16_t ack_timeout_us);
/* 仅初始化一条软 I2C（只配置该 bus 的引脚与 ctx）；其它 bus 禁止 soft_i2c_* 访问直至 init_all */
AppRet soft_i2c_init_single_bus(const SoftI2CBusPins *pins, uint8_t bus_id, uint16_t delay_us,
                               uint16_t ack_timeout_us);
/* 基础 I2C 原语。 */
AppRet soft_i2c_start(uint8_t bus_id);
AppRet soft_i2c_stop(uint8_t bus_id);
AppRet soft_i2c_write_byte(uint8_t bus_id, uint8_t data, bool *acked);
AppRet soft_i2c_read_byte(uint8_t bus_id, bool send_ack, uint8_t *out);
/* 常规单总线事务接口。 */
AppRet soft_i2c_write(uint8_t bus_id, uint8_t addr7, const uint8_t *buf, uint8_t len);
AppRet soft_i2c_write_read(uint8_t bus_id, uint8_t addr7, const uint8_t *wbuf, uint8_t wlen,
                           uint8_t *rbuf, uint8_t rlen);
/* 针对 MLX 热路径优化的“写 1 字节命令 + 读 N 字节”快捷接口。 */
AppRet soft_i2c_write_read_u8(uint8_t bus_id, uint8_t addr7, uint8_t wbyte, uint8_t *rbuf,
                              uint8_t rlen);
/* 多总线锁步访问：
 * 约束是这些总线上的目标器件必须是同一个 7-bit 地址，这样才能并行 bit-bang。
 */
void soft_i2c_write_read_lockstep_same_addr(uint16_t bus_mask, uint8_t addr7, const uint8_t *wbuf,
                                            uint8_t wlen, uint8_t *rbuf_by_bus, uint8_t rlen,
                                            AppRet *ret_by_bus);
/* 总线恢复：在怀疑 SDA 被拉住时发 9 个 SCL 脉冲，再补 STOP。 */
AppRet soft_i2c_bus_recover(uint8_t bus_id);

#endif /* SOFT_I2C_H */
