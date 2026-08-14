/**
 * @file sensor_app.c
 * @brief 三颗 MLX90393 的原始采集、USART1 IN 口转发和 UART DMA 上报编排。
 * @details 所属层级：APP。当前应用不执行磁干扰判断、姿态融合或其他复杂数据处理，
 *          只维护采样时序、当前 GAIN_SEL 和原始数据帧。
 */

#include "sensor_app.h"

#include <string.h>
#include <stdio.h>
#include "board_config.h"
#include "bsp_handles.h"
#include "bsp_time.h"
#include "bsp_uart_in.h"
#include "bsp_uart_dma.h"
#include "lsm6dsow.h"
#include "mlx90393.h"
#include "sensor_config.h"
#include "sensor_protocol.h"

/** @brief UART DMA 帧缓冲区容量，可容纳当前帧和 IN 口转发帧。 */
#define SENSOR_TX_CAPACITY 96u

/* 静态对象覆盖整个固件生命周期，避免裸机工程引入动态内存。 */
#if SENSOR_OUTPUT_ENABLE_MLX
static Mlx90393 g_mlx[BOARD_MLX_COUNT];
#endif
#if SENSOR_OUTPUT_ENABLE_IMU
static Lsm6dsow g_imu[BOARD_IMU_COUNT];
#endif
static SensorProtocolInput g_protocol_input;
static uint8_t g_tx_frame[SENSOR_TX_CAPACITY];
#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND == 0u)
static uint32_t g_next_sample_tick;
static uint8_t g_measurement_started_mask;
#else
static uint16_t g_fixed_send_divider_count;
static uint8_t g_mlx_runtime_fail_count[BOARD_MLX_COUNT];
static uint32_t g_mlx_next_retry_tick[BOARD_MLX_COUNT];
#endif

#if (BOARD_ENABLE_USART1_IN != 0u)
/* 三块及以上串联时，本机帧和 IN 口帧共用同一个 OUT 队列，严格按入队先后发送。 */
#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND != 0u)
#define SENSOR_OUT_QUEUE_DEPTH 32u
#else
#define SENSOR_OUT_QUEUE_DEPTH 8u
#endif

static uint8_t g_in_parse_frame[SENSOR_TX_CAPACITY];
static uint8_t g_out_dma_frame[SENSOR_TX_CAPACITY];
static uint8_t g_out_queue_frames[SENSOR_OUT_QUEUE_DEPTH][SENSOR_TX_CAPACITY];
static uint16_t g_out_queue_lengths[SENSOR_OUT_QUEUE_DEPTH];
static uint16_t g_in_offset;
static uint16_t g_in_expected_length;
static uint8_t g_out_queue_head;
static uint8_t g_out_queue_tail;
static uint8_t g_out_queue_count;
static uint8_t g_out_tx_active;
static volatile uint32_t g_out_queue_drop_count;
static volatile uint32_t g_out_queue_coalesce_count;
static volatile uint32_t g_out_queue_evict_count;

static uint16_t read_u16_be(const uint8_t *buffer, uint16_t offset) {
  return (uint16_t)(((uint16_t)buffer[offset] << 8) | buffer[offset + 1u]);
}

static void reset_in_parser(void) {
  g_in_offset = 0u;
  g_in_expected_length = 0u;
}

static uint8_t next_queue_index(uint8_t index) {
  index++;
  return (index >= SENSOR_OUT_QUEUE_DEPTH) ? 0u : index;
}

static uint8_t previous_queue_index(uint8_t index) {
  return (index == 0u) ? (SENSOR_OUT_QUEUE_DEPTH - 1u) : (uint8_t)(index - 1u);
}

static uint8_t same_frame_source(const uint8_t *a, uint16_t a_length,
                                 const uint8_t *b, uint16_t b_length) {
  if ((a == NULL) || (b == NULL) ||
      (a_length < SENSOR_PROTOCOL_MIN_FRAME_SIZE) ||
      (b_length < SENSOR_PROTOCOL_MIN_FRAME_SIZE)) {
    return 0u;
  }
  return ((a[2] == b[2]) && (a[3] == b[3]) && (a[4] == b[4])) ? 1u : 0u;
}

static uint8_t find_replaceable_same_source(const uint8_t *frame,
                                            uint16_t length,
                                            uint8_t *slot) {
  uint8_t index = g_out_queue_tail;
  for (uint8_t n = 0u; n < g_out_queue_count; ++n) {
    if (same_frame_source(g_out_queue_frames[index],
                          g_out_queue_lengths[index],
                          frame, length) != 0u) {
      *slot = index;
      return 1u;
    }
    index = next_queue_index(index);
  }
  return 0u;
}

static void remove_queued_frame(uint8_t slot) {
  uint8_t index;
  uint8_t next;

  if (g_out_queue_count == 0u) return;

  index = slot;
  next = next_queue_index(index);
  while (next != g_out_queue_head) {
    memcpy(g_out_queue_frames[index], g_out_queue_frames[next],
           g_out_queue_lengths[next]);
    g_out_queue_lengths[index] = g_out_queue_lengths[next];
    index = next;
    next = next_queue_index(next);
  }

  g_out_queue_head = previous_queue_index(g_out_queue_head);
  g_out_queue_count--;
}

static uint8_t evict_oldest_replaceable_frame(void) {
  if (g_out_queue_count == 0u) return 0u;

  remove_queued_frame(g_out_queue_tail);
  g_out_queue_evict_count++;
  return 1u;
}

static uint8_t enqueue_out_frame(const uint8_t *frame, uint16_t length) {
  uint8_t slot;
  if ((frame == NULL) || (length == 0u) || (length > SENSOR_TX_CAPACITY)) {
    return 0u;
  }
  if (find_replaceable_same_source(frame, length, &slot) != 0u) {
    memcpy(g_out_queue_frames[slot], frame, length);
    g_out_queue_lengths[slot] = length;
    g_out_queue_coalesce_count++;
    return 1u;
  }
  if (g_out_queue_count >= SENSOR_OUT_QUEUE_DEPTH) {
    if (evict_oldest_replaceable_frame() == 0u) {
      g_out_queue_drop_count++;
      return 0u;
    }
  }

  memcpy(g_out_queue_frames[g_out_queue_head], frame, length);
  g_out_queue_lengths[g_out_queue_head] = length;
  g_out_queue_head = next_queue_index(g_out_queue_head);
  g_out_queue_count++;
  return 1u;
}

static void feed_in_byte(uint8_t byte) {
  if (g_in_offset == 0u) {
    if (byte != SENSOR_PROTOCOL_HEADER_0) return;
    g_in_parse_frame[g_in_offset++] = byte;
    return;
  }

  if (g_in_offset == 1u) {
    if (byte != SENSOR_PROTOCOL_HEADER_1) {
      reset_in_parser();
      if (byte == SENSOR_PROTOCOL_HEADER_0) {
        g_in_parse_frame[g_in_offset++] = byte;
      }
      return;
    }
    g_in_parse_frame[g_in_offset++] = byte;
    return;
  }

  if (g_in_offset >= SENSOR_TX_CAPACITY) {
    reset_in_parser();
    return;
  }
  g_in_parse_frame[g_in_offset++] = byte;

  if (g_in_offset == 8u) {
    g_in_expected_length = read_u16_be(g_in_parse_frame, 6u);
    if ((g_in_expected_length < SENSOR_PROTOCOL_MIN_FRAME_SIZE) ||
        (g_in_expected_length > SENSOR_TX_CAPACITY)) {
      reset_in_parser();
    }
    return;
  }

  if ((g_in_expected_length != 0u) && (g_in_offset == g_in_expected_length)) {
    if (SensorProtocol_IsValidFrame(g_in_parse_frame, g_in_expected_length) != 0u) {
      (void)enqueue_out_frame(g_in_parse_frame, g_in_expected_length);
    }
    reset_in_parser();
  }
}

static void process_out_queue(void) {
  if ((g_out_tx_active != 0u) && (BspUartDma_IsBusy() == 0u)) {
    g_out_tx_active = 0u;
  }

  if ((g_out_tx_active == 0u) &&
      (g_out_queue_count != 0u) &&
      (BspUartDma_IsBusy() == 0u)) {
    uint16_t length = g_out_queue_lengths[g_out_queue_tail];
    memcpy(g_out_dma_frame, g_out_queue_frames[g_out_queue_tail], length);
    AppStatus status = BspUartDma_Transmit(g_out_dma_frame, length);
    if (status == APP_STATUS_OK) {
      remove_queued_frame(g_out_queue_tail);
      g_out_tx_active = 1u;
    } else if (status != APP_STATUS_BUSY) {
      remove_queued_frame(g_out_queue_tail);
    }
  }
}

static void process_uart_in(void) {
  uint8_t byte;
  uint16_t budget = BOARD_UART_IN_PROCESS_BYTE_BUDGET;

  while ((budget != 0u) && (BspUartIn_ReadByte(&byte) != 0u)) {
    feed_in_byte(byte);
    budget--;
  }
}
#endif

static void report_mlx_init(uint8_t index, const Mlx90393 *device, uint8_t ack,
                            AppStatus status) {
  char line[72];
  int length = snprintf(line, sizeof(line),
                        "MLX%u ADDR=0x%02X ACK=%u INIT=%u STEP=%u STATUS=0x%02X\r\n",
                        (unsigned)(index + 1u), (unsigned)device->address,
                        (unsigned)ack, (unsigned)status,
                        (unsigned)device->last_init_step,
                        (unsigned)device->last_status);
  if (length > 0) {
    if (length > (int)sizeof(line)) length = (int)sizeof(line);
    (void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)line, (uint16_t)length, 100u);
  }
}

static void report_i2c_scan(void) {
  char line[64];
  uint8_t ack[4];
  for (uint8_t i = 0u; i < 4u; ++i) {
    uint8_t address = (uint8_t)(0x0Cu + i);
    ack[i] = (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1),
                                     2u, BOARD_I2C_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
  }
  int length = snprintf(line, sizeof(line), "I2C_SCAN 0C=%u 0D=%u 0E=%u 0F=%u\r\n",
                        (unsigned)ack[0], (unsigned)ack[1],
                        (unsigned)ack[2], (unsigned)ack[3]);
  if (length > 0) {
    (void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)line, (uint16_t)length, 100u);
  }
}

/**
 * @brief 向所有已初始化的 MLX90393 启动一次 XYZT 转换。
 * @return 无返回值；各器件启动结果写入 g_measurement_started_mask。
 * @details 下一周期只读取启动成功的器件，避免把旧转换结果误标成新数据。
 */
#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND == 0u)
static void start_all_mlx(void) {
  g_measurement_started_mask = 0u;
#if SENSOR_OUTPUT_ENABLE_MLX
  for (uint8_t i = 0u; i < BOARD_MLX_COUNT; ++i) {
    if (Mlx90393_StartSingle(&g_mlx[i]) == APP_STATUS_OK) {
      g_measurement_started_mask |= (uint8_t)(1u << i);
    }
  }
#endif
}
#endif

/** @brief 向所有已初始化的 MLX90393 启动连续 burst 转换。 */
#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND != 0u)
static void start_all_mlx_burst(void) {
#if SENSOR_OUTPUT_ENABLE_MLX
  for (uint8_t i = 0u; i < BOARD_MLX_COUNT; ++i) {
    (void)Mlx90393_StartBurst(&g_mlx[i]);
  }
#endif
}
#endif

/** @brief 编码当前协议快照，并按当前方案发送或入队。 */
static void send_current_protocol_frame(void) {
  uint16_t frame_length = SensorProtocol_Encode(&g_protocol_input, g_tx_frame,
                                                sizeof(g_tx_frame));
  if (frame_length == 0u) return;
#if (BOARD_ENABLE_USART1_IN != 0u)
  (void)enqueue_out_frame(g_tx_frame, frame_length);
  process_out_queue();
#else
  if (BspUartDma_IsBusy() == 0u) {
    (void)BspUartDma_Transmit(g_tx_frame, frame_length);
  }
#endif
}

/**
 * @brief 初始化三颗磁传感器和周期采样状态。
 * @return 无返回值；单颗器件初始化失败不会阻止其他器件初始化。
 * @details 默认启动 MLX burst 连续转换；旧 10 ms 分支才启动第一轮单次测量。
 *          IMU 只有在后续显式打开输出宏时才会参与。
 */
void SensorApp_Init(void) {
  /* 清空协议快照和调度状态，保证重复初始化不会带入旧数据。 */
  memset(&g_protocol_input, 0, sizeof(g_protocol_input));
#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND == 0u)
  g_measurement_started_mask = 0u;
#else
  g_fixed_send_divider_count = 0u;
  memset(g_mlx_runtime_fail_count, 0, sizeof(g_mlx_runtime_fail_count));
  memset(g_mlx_next_retry_tick, 0, sizeof(g_mlx_next_retry_tick));
#endif
#if (BOARD_ENABLE_USART1_IN != 0u)
  reset_in_parser();
  g_out_queue_head = 0u;
  g_out_queue_tail = 0u;
  g_out_queue_count = 0u;
  g_out_tx_active = 0u;
  g_out_queue_drop_count = 0u;
  g_out_queue_coalesce_count = 0u;
  g_out_queue_evict_count = 0u;
#endif
  report_i2c_scan();

  /* 三颗 MLX 按板级地址表独立初始化。 */
#if SENSOR_OUTPUT_ENABLE_MLX
  for (uint8_t i = 0u; i < BOARD_MLX_COUNT; ++i) {
    uint8_t ack = (HAL_I2C_IsDeviceReady(&hi2c1,
                                          (uint16_t)(g_board_mlx_addresses[i] << 1),
                                          2u, BOARD_I2C_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
    AppStatus status = Mlx90393_Init(&g_mlx[i], g_board_mlx_addresses[i],
                                     &g_sensor_mlx_config);
    report_mlx_init(i, &g_mlx[i], ack, status);
  }
#endif

  /* 指尖硬件保留两颗 IMU，但当前默认不输出；指腹磁铁方案没有 IMU。 */
#if SENSOR_OUTPUT_ENABLE_IMU
  for (uint8_t i = 0u; i < BOARD_IMU_COUNT; ++i) {
    (void)Lsm6dsow_Init(&g_imu[i], g_board_imu_addresses[i], &g_sensor_imu_config);
  }
#endif

  /* 按编译宏选择 burst 连续转换，或保留原来的首轮 SM 流水转换。 */
#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND != 0u)
  start_all_mlx_burst();
  BspTime_ResetFrameTick();
#else
  start_all_mlx();
  g_next_sample_tick = BspTime_GetTickMs() + BOARD_SAMPLE_PERIOD_MS;
#endif
}

/**
 * @brief 周期读取全部传感器原始值，并在 UART DMA 空闲时发送一帧。
 * @return 无返回值；未到采样时刻时立即返回。
 * @details 默认分支从 MLX burst mode 读取最新结果；旧 10 ms 分支使用“读取上一轮、启动下一轮”的流水方式。
 *          IMU 仅在输出宏打开时读取。
 *          指腹磁铁方案把 USART1 IN 口帧和本机帧放入同一个 OUT 队列，按先到先发。
 */
void SensorApp_Process(void) {
  uint32_t now = BspTime_GetTickMs();

#if (BOARD_ENABLE_USART1_IN != 0u)
  process_out_queue();
  process_uart_in();
  process_out_queue();
#endif

#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND != 0u)
  if (BspTime_ConsumeFrameTick() == 0u) return;
  g_fixed_send_divider_count++;
  if (g_fixed_send_divider_count < BOARD_TIM2_FIXED_SEND_DIVIDER) return;
  g_fixed_send_divider_count = 0u;

  g_protocol_input.tick_ms = now;
  g_protocol_input.gain_sel = 0u;
#if SENSOR_OUTPUT_ENABLE_MLX
  g_protocol_input.gain_sel = g_mlx[0].config.gain_sel;
  for (uint8_t i = 0u; i < BOARD_MLX_COUNT; ++i) {
    Mlx90393Sample sample;
    if ((g_mlx_runtime_fail_count[i] >= 3u) &&
        ((int32_t)(now - g_mlx_next_retry_tick[i]) < 0)) {
      continue;
    }
    if (Mlx90393_Read(&g_mlx[i], &sample) == APP_STATUS_OK) {
      g_protocol_input.mlx[i] = sample;
      g_mlx_runtime_fail_count[i] = 0u;
    } else if (g_mlx_runtime_fail_count[i] < 255u) {
      g_mlx_runtime_fail_count[i]++;
      if (g_mlx_runtime_fail_count[i] >= 3u) {
        g_mlx_next_retry_tick[i] = now + BOARD_MLX_RUNTIME_FAIL_COOLDOWN_MS;
      }
    }
  }
#endif

#if SENSOR_OUTPUT_ENABLE_IMU
  for (uint8_t i = 0u; i < BOARD_IMU_COUNT; ++i) {
    (void)Lsm6dsow_Read(&g_imu[i], &g_protocol_input.imu[i]);
  }
#endif

#if (BOARD_ENABLE_USART1_IN != 0u)
  process_out_queue();
  process_uart_in();
  process_out_queue();
#endif
  send_current_protocol_frame();
#else
  /* 有符号 tick 差可兼容 32 位毫秒计数自然回绕。 */
  if ((int32_t)(now - g_next_sample_tick) < 0) return;
  g_next_sample_tick += BOARD_SAMPLE_PERIOD_MS;
  if ((int32_t)(now - g_next_sample_tick) >= 0) {
    /* 主循环严重滞后时从当前时刻重新排期，避免连续补采造成总线拥塞。 */
    g_next_sample_tick = now + BOARD_SAMPLE_PERIOD_MS;
  }

  /* 新周期先清空原始数组，失败器件不会沿用上一周期数据。 */
#if SENSOR_OUTPUT_ENABLE_MLX
  memset(&g_protocol_input.mlx, 0, sizeof(g_protocol_input.mlx));
#endif
#if SENSOR_OUTPUT_ENABLE_IMU
  memset(&g_protocol_input.imu, 0, sizeof(g_protocol_input.imu));
#endif
  g_protocol_input.tick_ms = now;
  g_protocol_input.gain_sel = 0u;
  g_mlx_frame_fresh_mask = 0u;
#if SENSOR_OUTPUT_ENABLE_MLX
  g_protocol_input.gain_sel = g_mlx[0].config.gain_sel;
#endif

  /* 读取上一周期成功启动的三颗 MLX；失败样本保持本周期预先清零值。 */
#if SENSOR_OUTPUT_ENABLE_MLX
  for (uint8_t n = 0u; n < BOARD_MLX_COUNT; ++n) {
    uint8_t i = n;
    if ((g_measurement_started_mask & (uint8_t)(1u << i)) != 0u) {
      (void)Mlx90393_Read(&g_mlx[i], &g_protocol_input.mlx[i]);
    }
  }
#endif

  /* 读取结束后立即启动下一轮转换，转换时间不阻塞本周期其余工作。 */
  start_all_mlx();

  /* 只有显式打开 IMU 输出时才读取温度、陀螺和加速度；失败样本保持零值。 */
#if SENSOR_OUTPUT_ENABLE_IMU
  for (uint8_t i = 0u; i < BOARD_IMU_COUNT; ++i) {
    (void)Lsm6dsow_Read(&g_imu[i], &g_protocol_input.imu[i]);
  }
#endif

#if (BOARD_ENABLE_USART1_IN != 0u)
  /* 采样期间到达的 IN 口帧先入同一个 OUT 队列，避免本机帧插队。 */
  process_uart_in();
#endif

  send_current_protocol_frame();
#endif
}

/** @brief 转发 LPUART1 DMA 发送完成事件，使下一帧可以复用发送缓冲区。 */
void SensorApp_OnUartTxComplete(void) {
  BspUartDma_TxComplete();
}

/** @brief 转发 LPUART1/DMA 错误事件，释放软件 busy 状态以便后续恢复。 */
void SensorApp_OnUartError(void) {
  BspUartDma_TxError();
}
