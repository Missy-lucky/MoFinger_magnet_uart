#include "app_config.h"
#include "soft_i2c.h"

#include <string.h>

/* 软 I2C 驱动：
 * 既支持普通单总线 I2C 事务，也支持当前项目特有的“多总线同地址锁步访问”。
 * 后者是 collector 提速的关键基础能力。
 */

static SoftI2CBusCtx g_i2c_ctx[SENSOR_BUS_COUNT];
/* 预先缓存 GPIO 端口与 BSRR mask，减少热路径中对 HAL 封装的依赖。 */
typedef struct {
  GPIO_TypeDef *sda_port;
  GPIO_TypeDef *scl_port;
  uint32_t sda_set_mask;
  uint32_t sda_reset_mask;
  uint32_t scl_set_mask;
  uint32_t scl_reset_mask;
  uint32_t sda_sample_mask;
} SoftI2CFastIo;

static SoftI2CFastIo g_i2c_fast_io[SENSOR_BUS_COUNT];
/* Cortex-M4 在 1us 内运行的核心时钟周期数。
 * 非 0 同时表示 DWT CYCCNT 已成功启用，可以作为软 I2C 的精确时间基准。 */
static uint32_t g_soft_i2c_cycles_per_us = 0u;
/* bit i=1 表示该 bus 已通过 init_all 或 init_single_bus 就绪，允许 I2C 访问 */
static uint16_t g_soft_i2c_bus_enabled_mask = 0u;


static bool is_valid_bus(uint8_t bus_id) { return bus_id < SENSOR_BUS_COUNT; }

/* 纯忙等延时内核。 */
static void delay_loops(uint32_t loops) {
  volatile uint32_t spin = loops;
  while (spin--) {
    __NOP();
  }
}

/* 启用 Cortex-M4 的 DWT 周期计数器，作为软 I2C 时钟的时间基准。
 *
 * 原实现用 C 空循环次数近似微秒，因此一旦编译优化从 -O0 改成 -O2，同样的循环次数会对应
 * 不同的真实时间，SCL 电气时序也会跟着改变。CYCCNT 直接累计 CPU 核心周期，所以 -O0/-O2
 * 下请求的 1us 都对应同样的真实周期数。后续用无符号减法计算时间差，即使 32 位计数器回绕，
 * 只要单次等待远小于一个回绕周期，结果仍然正确。 */
static void soft_i2c_init_cycle_timer(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u) {
    g_soft_i2c_cycles_per_us = SystemCoreClock / 1000000u;
  } else {
    /* 少数受限调试环境可能禁止 DWT。此时保持 0，让 delay_us_soft() 选择旧空循环兜底，
     * 而不是在没有任何延时的情况下错误运行 I2C。 */
    g_soft_i2c_cycles_per_us = 0u;
  }
}

/* 直接访问 GPIO BSRR/IDR 的快速 I/O 原语。 */
static void sda_write_fast(const SoftI2CFastIo *io, GPIO_PinState state) {
  io->sda_port->BSRR = (state == GPIO_PIN_SET) ? io->sda_set_mask : io->sda_reset_mask;
}

static void scl_write_fast(const SoftI2CFastIo *io, GPIO_PinState state) {
  io->scl_port->BSRR = (state == GPIO_PIN_SET) ? io->scl_set_mask : io->scl_reset_mask;
}

static GPIO_PinState sda_read_fast(const SoftI2CFastIo *io) {
  return ((io->sda_port->IDR & io->sda_sample_mask) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* 判断某条总线当前是否允许访问。 */
static bool bus_i2c_enabled(uint8_t bus_id) {
  if (!is_valid_bus(bus_id)) {
    return false;
  }
  return (g_soft_i2c_bus_enabled_mask & (uint16_t)(1u << bus_id)) != 0u;
}

/* 将“逻辑上的 us”转换成软延时循环次数。 */
static void delay_us_soft(uint16_t us) {
  if (g_soft_i2c_cycles_per_us != 0u) {
    /* 记录起点后等待指定周期数。这里不比较绝对结束值，避免 CYCCNT 回绕时判断失效。 */
    uint32_t start_cycle = DWT->CYCCNT;
    uint32_t wait_cycles = (uint32_t)us * g_soft_i2c_cycles_per_us;

    while ((uint32_t)(DWT->CYCCNT - start_cycle) < wait_cycles) {
      __NOP();
    }
    return;
  }

  delay_loops((uint32_t)us * (uint32_t)APP_I2C_SOFT_LOOP_MUL);
}

/* 初始化某条总线的快速 GPIO 缓存。 */
static void init_fast_io(uint8_t bus_id) {
  g_i2c_fast_io[bus_id].sda_port = g_i2c_ctx[bus_id].pins.sda_port;
  g_i2c_fast_io[bus_id].scl_port = g_i2c_ctx[bus_id].pins.scl_port;
  g_i2c_fast_io[bus_id].sda_set_mask = (uint32_t)g_i2c_ctx[bus_id].pins.sda_pin;
  g_i2c_fast_io[bus_id].sda_reset_mask = (uint32_t)g_i2c_ctx[bus_id].pins.sda_pin << 16U;
  g_i2c_fast_io[bus_id].scl_set_mask = (uint32_t)g_i2c_ctx[bus_id].pins.scl_pin;
  g_i2c_fast_io[bus_id].scl_reset_mask = (uint32_t)g_i2c_ctx[bus_id].pins.scl_pin << 16U;
  g_i2c_fast_io[bus_id].sda_sample_mask = (uint32_t)g_i2c_ctx[bus_id].pins.sda_pin;
}

/* 兼容式单总线 GPIO 包装。 */
static void sda_write(uint8_t bus_id, GPIO_PinState state) {
  sda_write_fast(&g_i2c_fast_io[bus_id], state);
}

static void scl_write(uint8_t bus_id, GPIO_PinState state) {
  scl_write_fast(&g_i2c_fast_io[bus_id], state);
}

static GPIO_PinState sda_read(uint8_t bus_id) {
  return sda_read_fast(&g_i2c_fast_io[bus_id]);
}

typedef struct {
  GPIO_TypeDef *port;
  uint32_t pin_mask;
} SoftI2CPortBatch;

/*
 * A lockstep transaction repeatedly drives the same set of buses for every
 * I2C bit.  The old path walked every bus and performed one BSRR write per
 * pin on every edge.  This plan converts that stable bus list into batches
 * keyed by GPIO port once, so the hot path only writes one combined mask per
 * port.  It changes CPU work between edges, not the electrical waveform:
 * callers still invoke the same START/STOP/ACK operations and delays.
 */
typedef struct {
  uint8_t valid;
  uint8_t bus_count;
  uint8_t bus_list[SENSOR_BUS_COUNT];
  uint8_t sda_count;
  uint8_t scl_count;
  SoftI2CPortBatch sda[SENSOR_BUS_COUNT];
  SoftI2CPortBatch scl[SENSOR_BUS_COUNT];
} SoftI2CWritePlan;

static SoftI2CWritePlan g_lockstep_write_plan;

/* Merge pins that share a GPIO port.  BSRR makes each merged update atomic. */
static void add_port_pin(SoftI2CPortBatch *batches, uint8_t *count,
                         GPIO_TypeDef *port, uint16_t pin) {
  for (uint8_t i = 0u; i < *count; i++) {
    if (batches[i].port == port) {
      batches[i].pin_mask |= (uint32_t)pin;
      return;
    }
  }

  if (*count < SENSOR_BUS_COUNT) {
    batches[*count].port = port;
    batches[*count].pin_mask = (uint32_t)pin;
    (*count)++;
  }
}

static const SoftI2CWritePlan *prepare_write_plan(const uint8_t *bus_list,
                                                   uint8_t bus_count) {
  SoftI2CWritePlan *plan = &g_lockstep_write_plan;

  /* Address groups reuse the same ordered list for many bytes and bits. */
  if ((plan->valid != 0u) && (plan->bus_count == bus_count) &&
      (memcmp(plan->bus_list, bus_list, bus_count) == 0)) {
    return plan;
  }

  /* A changed alive-mask/list can happen after an ACK failure, so rebuild it
   * before the next edge rather than accidentally driving a failed bus. */
  memset(plan, 0, sizeof(*plan));
  plan->bus_count = bus_count;
  memcpy(plan->bus_list, bus_list, bus_count);
  for (uint8_t i = 0u; i < bus_count; i++) {
    uint8_t bus_id = bus_list[i];
    add_port_pin(plan->sda, &plan->sda_count, g_i2c_fast_io[bus_id].sda_port,
                 g_i2c_ctx[bus_id].pins.sda_pin);
    add_port_pin(plan->scl, &plan->scl_count, g_i2c_fast_io[bus_id].scl_port,
                 g_i2c_ctx[bus_id].pins.scl_pin);
  }
  plan->valid = 1u;
  return plan;
}

static void write_port_batches(const SoftI2CPortBatch *batches, uint8_t count,
                               GPIO_PinState state) {
  for (uint8_t i = 0u; i < count; i++) {
    uint32_t mask = batches[i].pin_mask;
    /* GPIO BSRR lower half sets pins; upper half resets pins.  A single
     * 32-bit write preserves the simultaneous-edge property of lockstep. */
    batches[i].port->BSRR = (state == GPIO_PIN_SET) ? mask : (mask << 16u);
  }
}

/* 在 bus_mask 上仅保留已启用的总线。 */
static uint16_t filter_enabled_bus_mask(uint16_t bus_mask) {
  uint16_t mask = 0u;

  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    uint16_t bit = (uint16_t)(1u << bus_id);
    if ((bus_mask & bit) == 0u) {
      continue;
    }
    if (bus_i2c_enabled(bus_id)) {
      mask |= bit;
    }
  }
  return mask;
}

/* mask 与列表之间的转换辅助函数。 */
static uint8_t build_bus_list_from_mask(uint16_t bus_mask, uint8_t *bus_list) {
  uint8_t count = 0u;

  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    if ((bus_mask & (uint16_t)(1u << bus_id)) != 0u) {
      bus_list[count++] = bus_id;
    }
  }
  return count;
}

static uint8_t first_bus_in_mask(uint16_t bus_mask) {
  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    if ((bus_mask & (uint16_t)(1u << bus_id)) != 0u) {
      return bus_id;
    }
  }
  return SENSOR_BUS_COUNT;
}

/* 锁步访问时，统一参考第一条有效总线的时序参数。 */
static void delay_us_soft_mask(uint16_t bus_mask) {
  uint8_t ref_bus = first_bus_in_mask(bus_mask);

  if (ref_bus >= SENSOR_BUS_COUNT) {
    return;
  }
  delay_us_soft(g_i2c_ctx[ref_bus].delay_us);
}

static uint16_t ack_timeout_us_mask(uint16_t bus_mask) {
  uint8_t ref_bus = first_bus_in_mask(bus_mask);

  if (ref_bus >= SENSOR_BUS_COUNT) {
    return 0u;
  }
  return g_i2c_ctx[ref_bus].ack_timeout_us;
}

/* 针对总线列表/掩码的批量 SDA/SCL 控制。 */
static void sda_write_list(const uint8_t *bus_list, uint8_t bus_count, GPIO_PinState state) {
  /* The cached plan is shared by SDA and SCL calls for this bus list. */
  const SoftI2CWritePlan *plan = prepare_write_plan(bus_list, bus_count);
  write_port_batches(plan->sda, plan->sda_count, state);
}

static void scl_write_list(const uint8_t *bus_list, uint8_t bus_count, GPIO_PinState state) {
  const SoftI2CWritePlan *plan = prepare_write_plan(bus_list, bus_count);
  write_port_batches(plan->scl, plan->scl_count, state);
}

static void sda_write_mask(uint16_t bus_mask, GPIO_PinState state) {
  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    if ((bus_mask & (uint16_t)(1u << bus_id)) != 0u) {
      sda_write(bus_id, state);
    }
  }
}

static void scl_write_mask(uint16_t bus_mask, GPIO_PinState state) {
  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    if ((bus_mask & (uint16_t)(1u << bus_id)) != 0u) {
      scl_write(bus_id, state);
    }
  }
}

/* 在多条总线上同时产生一个 START。 */
static void soft_i2c_start_mask(uint16_t bus_mask) {
  uint8_t bus_list[SENSOR_BUS_COUNT];
  uint8_t bus_count = build_bus_list_from_mask(bus_mask, bus_list);
  uint16_t delay_us;

  /* 没有任何有效总线时，整次锁步 START 直接跳过。 */
  if (bus_count == 0u) {
    return;
  }

  /* 锁步总线共享同一套时序参数，因此取第一条有效总线作参考。 */
  delay_us = g_i2c_ctx[bus_list[0]].delay_us;
  /* 标准 I2C START：SDA 高 -> SCL 高 -> SDA 拉低 -> SCL 拉低。 */
  sda_write_list(bus_list, bus_count, GPIO_PIN_SET);
  scl_write_list(bus_list, bus_count, GPIO_PIN_SET);
  delay_us_soft(delay_us);
  sda_write_list(bus_list, bus_count, GPIO_PIN_RESET);
  delay_us_soft(delay_us);
  scl_write_list(bus_list, bus_count, GPIO_PIN_RESET);
  delay_us_soft(delay_us);
}

/* 在多条总线上同时产生一个 STOP。 */
static void soft_i2c_stop_mask(uint16_t bus_mask) {
  uint8_t bus_list[SENSOR_BUS_COUNT];
  uint8_t bus_count = build_bus_list_from_mask(bus_mask, bus_list);
  uint16_t delay_us;

  if (bus_count == 0u) {
    return;
  }

  delay_us = g_i2c_ctx[bus_list[0]].delay_us;
  /* 标准 I2C STOP：SDA 低 -> SCL 高 -> SDA 高。 */
  sda_write_list(bus_list, bus_count, GPIO_PIN_RESET);
  delay_us_soft(delay_us);
  scl_write_list(bus_list, bus_count, GPIO_PIN_SET);
  delay_us_soft(delay_us);
  sda_write_list(bus_list, bus_count, GPIO_PIN_SET);
  delay_us_soft(delay_us);
}

/* 锁步事务中，失败总线统一补 STOP，并标记错误。 */
static void lockstep_stop_failed_buses(uint16_t failed_mask, AppRet *ret_by_bus) {
  soft_i2c_stop_mask(failed_mask);
  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    if ((failed_mask & (uint16_t)(1u << bus_id)) == 0u) {
      continue;
    }
    if (ret_by_bus != 0) {
      ret_by_bus[bus_id] = APP_ERR_TIMEOUT;
    }
  }
}

/* 在所有 still-alive 总线上同时写出同一个字节，并独立等待各自 ACK。 */
static void lockstep_write_shared_byte(uint16_t *alive_mask, uint8_t data, AppRet *ret_by_bus) {
  uint16_t bus_mask = *alive_mask;
  uint8_t bus_list[SENSOR_BUS_COUNT];
  uint8_t bus_count;
  uint16_t pending_ack_mask;
  uint16_t failed_mask;
  uint16_t delay_us;
  uint16_t timeout;

  if (bus_mask == 0u) {
    return;
  }
  bus_count = build_bus_list_from_mask(bus_mask, bus_list);
  if (bus_count == 0u) {
    return;
  }
  /* 同一批 still-alive 总线共用一个 delay_us。 */
  delay_us = g_i2c_ctx[bus_list[0]].delay_us;
  for (uint8_t i = 0u; i < 8u; i++) {
    /* 所有总线同时打一位数据，再一起拉高/拉低 SCL。 */
    sda_write_list(bus_list, bus_count, (data & 0x80u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    delay_us_soft(delay_us);
    scl_write_list(bus_list, bus_count, GPIO_PIN_SET);
    delay_us_soft(delay_us);
    scl_write_list(bus_list, bus_count, GPIO_PIN_RESET);
    delay_us_soft(delay_us);
    data <<= 1;
  }

  sda_write_list(bus_list, bus_count, GPIO_PIN_SET);
  delay_us_soft(delay_us);
  scl_write_list(bus_list, bus_count, GPIO_PIN_SET);

  /* ACK 需要按总线逐条判断，因此用 pending_ack_mask 跟踪“还有谁没应答”。 */
  pending_ack_mask = bus_mask;
  timeout = g_i2c_ctx[bus_list[0]].ack_timeout_us;
  while ((pending_ack_mask != 0u) && (timeout > 0u)) {
    for (uint8_t i = 0u; i < bus_count; i++) {
      uint8_t bus_id = bus_list[i];
      uint16_t bit = (uint16_t)(1u << bus_id);
      if ((pending_ack_mask & bit) == 0u) {
        continue;
      }
      if (sda_read(bus_id) == GPIO_PIN_RESET) {
        /* 某条总线拉低 SDA，说明它已经 ACK，可以从等待集合中移除。 */
        pending_ack_mask &= (uint16_t)(~bit);
      }
    }
    if (pending_ack_mask != 0u) {
      delay_us_soft(1u);
      timeout--;
    }
  }

  if (pending_ack_mask == 0u) {
    delay_us_soft(delay_us);
  }
  scl_write_list(bus_list, bus_count, GPIO_PIN_RESET);
  delay_us_soft(delay_us);

  failed_mask = pending_ack_mask;
  if (failed_mask != 0u) {
    /* 把失败总线从 still-alive 集合里剔除，后续锁步访问不再带上它。 */
    *alive_mask = (uint16_t)(bus_mask & (uint16_t)(~failed_mask));
    lockstep_stop_failed_buses(failed_mask, ret_by_bus);
  }
}

/* 在 still-alive 总线上同时读取一个字节。 */
static void lockstep_read_bytes(uint16_t bus_mask, bool send_ack, uint8_t *out_by_bus) {
  uint8_t bus_list[SENSOR_BUS_COUNT];
  uint8_t bus_count = build_bus_list_from_mask(bus_mask, bus_list);
  uint16_t delay_us;

  for (uint8_t i = 0u; i < bus_count; i++) {
    /* 先清零本轮要读出的结果缓冲。 */
    out_by_bus[bus_list[i]] = 0u;
  }

  if (bus_count == 0u) {
    return;
  }
  delay_us = g_i2c_ctx[bus_list[0]].delay_us;
  sda_write_list(bus_list, bus_count, GPIO_PIN_SET);
  for (uint8_t i = 0u; i < 8u; i++) {
    /* 所有总线先整体左移一位，为即将采样到的新 bit 腾位置。 */
    for (uint8_t j = 0u; j < bus_count; j++) {
      out_by_bus[bus_list[j]] <<= 1;
    }
    scl_write_list(bus_list, bus_count, GPIO_PIN_SET);
    delay_us_soft(delay_us);
    for (uint8_t j = 0u; j < bus_count; j++) {
      uint8_t bus_id = bus_list[j];
      if (sda_read(bus_id) == GPIO_PIN_SET) {
        /* 哪条总线此刻读到高电平，就把对应 bit 补成 1。 */
        out_by_bus[bus_id] |= 1u;
      }
    }
    scl_write_list(bus_list, bus_count, GPIO_PIN_RESET);
    delay_us_soft(delay_us);
  }

  /* 读完 8bit 后，统一发 ACK 或最后一个字节的 NACK。 */
  sda_write_list(bus_list, bus_count, send_ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
  delay_us_soft(delay_us);
  scl_write_list(bus_list, bus_count, GPIO_PIN_SET);
  delay_us_soft(delay_us);
  scl_write_list(bus_list, bus_count, GPIO_PIN_RESET);
  sda_write_list(bus_list, bus_count, GPIO_PIN_SET);
}

/* 单总线 ACK 等待快速路径。 */
static AppRet wait_ack_bit_fast(const SoftI2CFastIo *io, uint16_t delay_us, uint16_t ack_timeout_us, bool *acked) {
  uint32_t delay_loops_count = (uint32_t)delay_us * (uint32_t)APP_I2C_SOFT_LOOP_MUL;
  uint16_t timeout = ack_timeout_us;

  /* 写完 8bit 后先释放 SDA，再拉高 SCL，进入 ACK 采样窗口。 */
  sda_write_fast(io, GPIO_PIN_SET);
  delay_loops(delay_loops_count);
  scl_write_fast(io, GPIO_PIN_SET);
  while (timeout--) {
    if (sda_read_fast(io) == GPIO_PIN_RESET) {
      /* 从机把 SDA 拉低，表示 ACK 成功。 */
      *acked = true;
      delay_loops(delay_loops_count);
      scl_write_fast(io, GPIO_PIN_RESET);
      delay_loops(delay_loops_count);
      return APP_OK;
    }
    delay_us_soft(1u);
  }

  *acked = false;
  /* 超时则认为本次事务失败，但仍要把 SCL 拉回低电平，收好时序。 */
  scl_write_fast(io, GPIO_PIN_RESET);
  delay_loops(delay_loops_count);
  return APP_ERR_TIMEOUT;
}

static AppRet wait_ack_bit(uint8_t bus_id, bool *acked) {
  return wait_ack_bit_fast(&g_i2c_fast_io[bus_id], g_i2c_ctx[bus_id].delay_us, g_i2c_ctx[bus_id].ack_timeout_us,
                           acked);
}

/* 单总线字节写/字节读快速路径。 */
static AppRet soft_i2c_write_byte_fast(uint8_t bus_id, uint8_t data, uint16_t delay_us, uint16_t ack_timeout_us,
                                       bool *acked) {
  const SoftI2CFastIo *io = &g_i2c_fast_io[bus_id];
  uint32_t delay_loops_count = (uint32_t)delay_us * (uint32_t)APP_I2C_SOFT_LOOP_MUL;

  for (uint8_t i = 0u; i < 8u; i++) {
    /* 最高位先发，符合 I2C MSB-first 规则。 */
    sda_write_fast(io, (data & 0x80u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    delay_loops(delay_loops_count);
    scl_write_fast(io, GPIO_PIN_SET);
    delay_loops(delay_loops_count);
    scl_write_fast(io, GPIO_PIN_RESET);
    delay_loops(delay_loops_count);
    data <<= 1;
  }

  return wait_ack_bit_fast(io, delay_us, ack_timeout_us, acked);
}

static void soft_i2c_read_byte_fast(uint8_t bus_id, uint16_t delay_us, bool send_ack, uint8_t *out) {
  const SoftI2CFastIo *io = &g_i2c_fast_io[bus_id];
  uint32_t delay_loops_count = (uint32_t)delay_us * (uint32_t)APP_I2C_SOFT_LOOP_MUL;
  uint8_t data = 0u;

  sda_write_fast(io, GPIO_PIN_SET);
  for (uint8_t bit_idx = 0u; bit_idx < 8u; bit_idx++) {
    data <<= 1;
    scl_write_fast(io, GPIO_PIN_SET);
    delay_loops(delay_loops_count);
    if (sda_read_fast(io) == GPIO_PIN_SET) {
      data |= 1u;
    }
    scl_write_fast(io, GPIO_PIN_RESET);
    delay_loops(delay_loops_count);
  }

  sda_write_fast(io, send_ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
  delay_loops(delay_loops_count);
  scl_write_fast(io, GPIO_PIN_SET);
  delay_loops(delay_loops_count);
  scl_write_fast(io, GPIO_PIN_RESET);
  sda_write_fast(io, GPIO_PIN_SET);
  *out = data;
}

/* 连续读多个字节的热路径，供 MLX RM 使用。 */
static void soft_i2c_read_bytes_fast(uint8_t bus_id, uint16_t delay_us, uint8_t *rbuf, uint8_t rlen) {
  const SoftI2CFastIo *io = &g_i2c_fast_io[bus_id];
  uint32_t delay_loops_count = (uint32_t)delay_us * (uint32_t)APP_I2C_SOFT_LOOP_MUL;

  sda_write_fast(io, GPIO_PIN_SET);
  for (uint8_t byte_idx = 0u; byte_idx < rlen; byte_idx++) {
    uint8_t data = 0u;

    for (uint8_t bit_idx = 0u; bit_idx < 8u; bit_idx++) {
      /* 每个 bit 都是：拉高 SCL 采样 -> 读 SDA -> 拉低 SCL。 */
      data <<= 1;
      scl_write_fast(io, GPIO_PIN_SET);
      delay_loops(delay_loops_count);
      if (sda_read_fast(io) == GPIO_PIN_SET) {
        data |= 1u;
      }
      scl_write_fast(io, GPIO_PIN_RESET);
      delay_loops(delay_loops_count);
    }

    /* 中间字节回 ACK，最后一个字节回 NACK。 */
    sda_write_fast(io, (byte_idx < (uint8_t)(rlen - 1u)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    delay_loops(delay_loops_count);
    scl_write_fast(io, GPIO_PIN_SET);
    delay_loops(delay_loops_count);
    scl_write_fast(io, GPIO_PIN_RESET);
    sda_write_fast(io, GPIO_PIN_SET);
    rbuf[byte_idx] = data;
  }
}

/* 初始化全部或单条软 I2C 总线。 */
AppRet soft_i2c_init_all(const SoftI2CBusPins *pins, uint8_t bus_count, uint16_t delay_us,
                         uint16_t ack_timeout_us) {
  uint8_t i;
  if ((pins == 0) || (bus_count != SENSOR_BUS_COUNT) || (delay_us == 0u) ||
      (ack_timeout_us == 0u)) {
    return APP_ERR_PARAM;
  }

  soft_i2c_init_cycle_timer();
  g_lockstep_write_plan.valid = 0u;
  for (i = 0; i < SENSOR_BUS_COUNT; i++) {
    g_i2c_ctx[i].pins = pins[i];
    g_i2c_ctx[i].delay_us = delay_us;
    g_i2c_ctx[i].ack_timeout_us = ack_timeout_us;
    init_fast_io(i);
    sda_write(i, GPIO_PIN_SET);
    scl_write(i, GPIO_PIN_SET);
  }
  if (SENSOR_BUS_COUNT >= 16) {
    g_soft_i2c_bus_enabled_mask = 0xFFFFu;
  } else {
    g_soft_i2c_bus_enabled_mask = (uint16_t)((1u << SENSOR_BUS_COUNT) - 1u);
  }
  return APP_OK;
}

AppRet soft_i2c_init_single_bus(const SoftI2CBusPins *pins, uint8_t bus_id, uint16_t delay_us,
                                uint16_t ack_timeout_us) {
  if ((pins == NULL) || (!is_valid_bus(bus_id)) || (delay_us == 0u) || (ack_timeout_us == 0u)) {
    return APP_ERR_PARAM;
  }

  soft_i2c_init_cycle_timer();
  g_lockstep_write_plan.valid = 0u;
  g_soft_i2c_bus_enabled_mask = 0u;
  g_i2c_ctx[bus_id].pins = pins[bus_id];
  g_i2c_ctx[bus_id].delay_us = delay_us;
  g_i2c_ctx[bus_id].ack_timeout_us = ack_timeout_us;
  init_fast_io(bus_id);
  sda_write(bus_id, GPIO_PIN_SET);
  scl_write(bus_id, GPIO_PIN_SET);
  g_soft_i2c_bus_enabled_mask = (uint16_t)(1u << bus_id);
  return APP_OK;
}

/* 基础 I2C 原语。 */
AppRet soft_i2c_start(uint8_t bus_id) {
  if ((!is_valid_bus(bus_id)) || (!bus_i2c_enabled(bus_id))) {
    return APP_ERR_PARAM;
  }
  /* 单总线 START 与锁步版本完全同构，只是对象缩成一条总线。 */
  sda_write(bus_id, GPIO_PIN_SET);
  scl_write(bus_id, GPIO_PIN_SET);
  delay_us_soft(g_i2c_ctx[bus_id].delay_us);
  sda_write(bus_id, GPIO_PIN_RESET);
  delay_us_soft(g_i2c_ctx[bus_id].delay_us);
  scl_write(bus_id, GPIO_PIN_RESET);
  delay_us_soft(g_i2c_ctx[bus_id].delay_us);
  return APP_OK;
}

AppRet soft_i2c_stop(uint8_t bus_id) {
  if ((!is_valid_bus(bus_id)) || (!bus_i2c_enabled(bus_id))) {
    return APP_ERR_PARAM;
  }
  /* 标准 I2C STOP：SDA 低 -> SCL 高 -> SDA 高。 */
  sda_write(bus_id, GPIO_PIN_RESET);
  delay_us_soft(g_i2c_ctx[bus_id].delay_us);
  scl_write(bus_id, GPIO_PIN_SET);
  delay_us_soft(g_i2c_ctx[bus_id].delay_us);
  sda_write(bus_id, GPIO_PIN_SET);
  delay_us_soft(g_i2c_ctx[bus_id].delay_us);
  return APP_OK;
}

AppRet soft_i2c_write_byte(uint8_t bus_id, uint8_t data, bool *acked) {
  if ((!is_valid_bus(bus_id)) || (!bus_i2c_enabled(bus_id)) || (acked == 0)) {
    return APP_ERR_PARAM;
  }

  return soft_i2c_write_byte_fast(bus_id, data, g_i2c_ctx[bus_id].delay_us, g_i2c_ctx[bus_id].ack_timeout_us,
                                  acked);
}

AppRet soft_i2c_read_byte(uint8_t bus_id, bool send_ack, uint8_t *out) {
  if ((!is_valid_bus(bus_id)) || (!bus_i2c_enabled(bus_id)) || (out == 0)) {
    return APP_ERR_PARAM;
  }

  soft_i2c_read_byte_fast(bus_id, g_i2c_ctx[bus_id].delay_us, send_ack, out);
  return APP_OK;
}

/* 常规单总线事务接口。 */
AppRet soft_i2c_write(uint8_t bus_id, uint8_t addr7, const uint8_t *buf, uint8_t len) {
  uint8_t i;
  bool acked = false;
  AppRet ret;
  if ((!is_valid_bus(bus_id)) || (buf == 0) || (len == 0u)) {
    return APP_ERR_PARAM;
  }

  ret = soft_i2c_start(bus_id);
  if (ret != APP_OK) {
    return ret;
  }

  /* 第一个字节固定是 7-bit 地址左移一位，再拼接写方向位 0。 */
  ret = soft_i2c_write_byte(bus_id, (uint8_t)(addr7 << 1), &acked);
  if ((ret != APP_OK) || (!acked)) {
    soft_i2c_stop(bus_id);
    return (ret == APP_OK) ? APP_ERR_NACK : ret;
  }

  for (i = 0u; i < len; i++) {
    /* 后续连续写出 payload。 */
    ret = soft_i2c_write_byte(bus_id, buf[i], &acked);
    if ((ret != APP_OK) || (!acked)) {
      soft_i2c_stop(bus_id);
      return (ret == APP_OK) ? APP_ERR_NACK : ret;
    }
  }

  soft_i2c_stop(bus_id);
  return APP_OK;
}

AppRet soft_i2c_write_read(uint8_t bus_id, uint8_t addr7, const uint8_t *wbuf, uint8_t wlen,
                           uint8_t *rbuf, uint8_t rlen) {
  uint8_t i;
  bool acked = false;
  AppRet ret;
  uint16_t delay_us;
  uint16_t ack_timeout_us;
  if ((!is_valid_bus(bus_id)) || (wbuf == 0) || (rbuf == 0) || (wlen == 0u) || (rlen == 0u)) {
    return APP_ERR_PARAM;
  }
  delay_us = g_i2c_ctx[bus_id].delay_us;
  ack_timeout_us = g_i2c_ctx[bus_id].ack_timeout_us;

  /* 第一段：地址+写方向，送出命令/寄存器选择字节。 */
  ret = soft_i2c_start(bus_id);
  if (ret != APP_OK) {
    return ret;
  }

  ret = soft_i2c_write_byte_fast(bus_id, (uint8_t)(addr7 << 1), delay_us, ack_timeout_us, &acked);
  if ((ret != APP_OK) || (!acked)) {
    soft_i2c_stop(bus_id);
    return (ret == APP_OK) ? APP_ERR_NACK : ret;
  }

  for (i = 0u; i < wlen; i++) {
    ret = soft_i2c_write_byte_fast(bus_id, wbuf[i], delay_us, ack_timeout_us, &acked);
    if ((ret != APP_OK) || (!acked)) {
      soft_i2c_stop(bus_id);
      return (ret == APP_OK) ? APP_ERR_NACK : ret;
    }
  }

  /* 第二段：重复 START，切换到读方向。 */
  ret = soft_i2c_start(bus_id);
  if (ret != APP_OK) {
    soft_i2c_stop(bus_id);
    return ret;
  }

  ret = soft_i2c_write_byte_fast(bus_id, (uint8_t)((addr7 << 1) | 0x01u), delay_us, ack_timeout_us, &acked);
  if ((ret != APP_OK) || (!acked)) {
    soft_i2c_stop(bus_id);
    return (ret == APP_OK) ? APP_ERR_NACK : ret;
  }

  /* 第三段：连续读回结果字节。 */
  soft_i2c_read_bytes_fast(bus_id, delay_us, rbuf, rlen);

  soft_i2c_stop(bus_id);
  return APP_OK;
}

/* 目前 MLX 热路径最常用的“写 1 字节命令 + 读 N 字节”快捷事务。 */
AppRet soft_i2c_write_read_u8(uint8_t bus_id, uint8_t addr7, uint8_t wbyte, uint8_t *rbuf,
                              uint8_t rlen) {
  bool acked = false;
  AppRet ret;
  uint16_t delay_us;
  uint16_t ack_timeout_us;

  if ((!is_valid_bus(bus_id)) || (rbuf == 0) || (rlen == 0u)) {
    return APP_ERR_PARAM;
  }
  delay_us = g_i2c_ctx[bus_id].delay_us;
  ack_timeout_us = g_i2c_ctx[bus_id].ack_timeout_us;

  /* 这是 write_read 的特化版本：写缓冲固定只有 1 个字节。 */
  ret = soft_i2c_start(bus_id);
  if (ret != APP_OK) {
    return ret;
  }

  ret = soft_i2c_write_byte_fast(bus_id, (uint8_t)(addr7 << 1), delay_us, ack_timeout_us, &acked);
  if ((ret != APP_OK) || (!acked)) {
    soft_i2c_stop(bus_id);
    return (ret == APP_OK) ? APP_ERR_NACK : ret;
  }

  ret = soft_i2c_write_byte_fast(bus_id, wbyte, delay_us, ack_timeout_us, &acked);
  if ((ret != APP_OK) || (!acked)) {
    soft_i2c_stop(bus_id);
    return (ret == APP_OK) ? APP_ERR_NACK : ret;
  }

  ret = soft_i2c_start(bus_id);
  if (ret != APP_OK) {
    soft_i2c_stop(bus_id);
    return ret;
  }

  ret = soft_i2c_write_byte_fast(bus_id, (uint8_t)((addr7 << 1) | 0x01u), delay_us, ack_timeout_us, &acked);
  if ((ret != APP_OK) || (!acked)) {
    soft_i2c_stop(bus_id);
    return (ret == APP_OK) ? APP_ERR_NACK : ret;
  }

  soft_i2c_read_bytes_fast(bus_id, delay_us, rbuf, rlen);

  soft_i2c_stop(bus_id);
  return APP_OK;
}

/* 多总线、同地址的锁步事务入口。 */
void soft_i2c_write_read_lockstep_same_addr(uint16_t bus_mask, uint8_t addr7, const uint8_t *wbuf,
                                            uint8_t wlen, uint8_t *rbuf_by_bus, uint8_t rlen,
                                            AppRet *ret_by_bus) {
  uint16_t alive_mask;
  uint8_t read_tmp[SENSOR_BUS_COUNT];
  uint8_t alive_bus_list[SENSOR_BUS_COUNT];
  uint8_t alive_bus_count;

  if (ret_by_bus != 0) {
    for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
      ret_by_bus[bus_id] = APP_ERR_PARAM;
    }
  }

  if ((wbuf == 0) || (rbuf_by_bus == 0) || (ret_by_bus == 0) || (wlen == 0u) || (rlen == 0u)) {
    return;
  }

  /* 先把无效/未启用总线过滤掉，避免后续锁步事务带上错误对象。 */
  alive_mask = filter_enabled_bus_mask(bus_mask);
  for (uint8_t bus_id = 0u; bus_id < SENSOR_BUS_COUNT; bus_id++) {
    uint16_t bit = (uint16_t)(1u << bus_id);
    if ((bus_mask & bit) != 0u) {
      ret_by_bus[bus_id] = ((alive_mask & bit) != 0u) ? APP_OK : APP_ERR_PARAM;
    }
  }
  if (alive_mask == 0u) {
    return;
  }

  /* 第一段：地址+写方向。 */
  soft_i2c_start_mask(alive_mask);
  lockstep_write_shared_byte(&alive_mask, (uint8_t)(addr7 << 1), ret_by_bus);
  if (alive_mask == 0u) {
    return;
  }

  for (uint8_t i = 0u; (i < wlen) && (alive_mask != 0u); i++) {
    /* 仍存活的总线一起写命令字节；中途失败的会被动态踢出。 */
    lockstep_write_shared_byte(&alive_mask, wbuf[i], ret_by_bus);
  }
  if (alive_mask == 0u) {
    return;
  }

  /* 第二段：重复 START，切换成读方向。 */
  soft_i2c_start_mask(alive_mask);
  lockstep_write_shared_byte(&alive_mask, (uint8_t)((addr7 << 1) | 0x01u), ret_by_bus);
  if (alive_mask == 0u) {
    return;
  }
  alive_bus_count = build_bus_list_from_mask(alive_mask, alive_bus_list);

  for (uint8_t i = 0u; i < rlen; i++) {
    /* 每次锁步读回一个字节，再按 bus_id 把结果散写到输出矩阵里。 */
    lockstep_read_bytes(alive_mask, i < (uint8_t)(rlen - 1u), read_tmp);
    for (uint8_t j = 0u; j < alive_bus_count; j++) {
      uint8_t bus_id = alive_bus_list[j];
      rbuf_by_bus[(bus_id * rlen) + i] = read_tmp[bus_id];
    }
  }

  /* still-alive 的总线在正常收尾时统一补 STOP，减少逐条串行收尾的额外开销。 */
  soft_i2c_stop_mask(alive_mask);
  for (uint8_t j = 0u; j < alive_bus_count; j++) {
    uint8_t bus_id = alive_bus_list[j];
    ret_by_bus[bus_id] = APP_OK;
  }
}

/* 简单总线恢复：发 9 个时钟并补 STOP。 */
AppRet soft_i2c_bus_recover(uint8_t bus_id) {
  uint8_t i;
  if ((!is_valid_bus(bus_id)) || (!bus_i2c_enabled(bus_id))) {
    return APP_ERR_PARAM;
  }

  sda_write(bus_id, GPIO_PIN_SET);
  for (i = 0; i < 9u; i++) {
    scl_write(bus_id, GPIO_PIN_SET);
    delay_us_soft(g_i2c_ctx[bus_id].delay_us);
    scl_write(bus_id, GPIO_PIN_RESET);
    delay_us_soft(g_i2c_ctx[bus_id].delay_us);
  }
  return soft_i2c_stop(bus_id);
}
