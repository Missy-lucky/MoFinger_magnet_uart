/**
 * @file board_config.h
 * @brief 板级地址、周期和 Flash 分区常量声明。
 * @details 所属层级：BSP。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

/*
 * 方案选择：
 *   0 = 指尖磁铁方案：不开 USART1 IN 口，传感器类型 0x01；当前固件不把 IMU 写入上报帧。
 *   1 = 指腹磁铁方案：开启 USART1 IN 口，传感器类型 0x02；该方案硬件上没有 IMU。
 *
 * 需要编译指腹固件时，把下面的 BOARD_ENABLE_USART1_IN 改为 1。
 */
#ifndef BOARD_ENABLE_USART1_IN
#define BOARD_ENABLE_USART1_IN 1u
#endif

/*
 * 发送节拍选择：
 *   0 = 保留原来的主循环 10 ms deadline 采样/发帧分支。
 *   1 = TIM2 配成 1000 Hz，每个 TIM2 tick 触发一次上报帧；MLX 使用 burst mode
 *       连续转换，若本 tick 未读到新样本则复用上一份样本形成重复帧。
 */
#ifndef BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND
#define BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND 1u
#endif

/*
 * 1000 Hz 固定节拍分支的 UART 上报分频：
 *   1 = 每个 TIM2 tick 都上报，约 1000 Hz，单板调试用。
 *   2 = 每 2 个 TIM2 tick 上报一次，约 500 Hz。
 *   4 = 每 4 个 TIM2 tick 上报一次，约 250 Hz，三块以上串联更稳。
 *
 * MLX burst mode 本身仍连续转换；该宏只限制 UART 帧输出频率。
 */
#ifndef BOARD_TIM2_FIXED_SEND_DIVIDER
#define BOARD_TIM2_FIXED_SEND_DIVIDER 2u
#endif

#define BOARD_MLX_COUNT 3u
#if (BOARD_ENABLE_USART1_IN != 0u)
#define BOARD_IMU_COUNT 0u
#else
#define BOARD_IMU_COUNT 2u
#endif

#define BOARD_SENSOR_TYPE_FINGERTIP_MAGNET 0x01u
#define BOARD_SENSOR_TYPE_FINGER_PULP_MAGNET 0x02u

/* 通信协议中的传感器域和传感器 id。 */
#define BOARD_SENSOR_DOMAIN 1u
#define BOARD_SENSOR_ID 0x00u
#if (BOARD_ENABLE_USART1_IN != 0u)
#define BOARD_SENSOR_TYPE BOARD_SENSOR_TYPE_FINGER_PULP_MAGNET
#else
#define BOARD_SENSOR_TYPE BOARD_SENSOR_TYPE_FINGERTIP_MAGNET
#endif

/* Schematic straps (A1/A0): U4=00, U6=01, U8=10.
 * Therefore the 7-bit I2C addresses are U4=0x0C, U6=0x0E, U8=0x0D. */
#define BOARD_MLX_PRIMARY_ADDR 0x0Cu
#define BOARD_MLX_INTERFERENCE_1_ADDR 0x0Eu
#define BOARD_MLX_INTERFERENCE_2_ADDR 0x0Du

/* LSM6DSOW SDO/SA0 straps: U1=0, U2=1. */
#define BOARD_IMU_1_ADDR 0x6Au
#define BOARD_IMU_2_ADDR 0x6Bu

#define BOARD_I2C_TIMEOUT_MS 20u
#define BOARD_I2C_RUNTIME_TIMEOUT_MS 2u
#define BOARD_MLX_RUNTIME_FAIL_COOLDOWN_MS 20u
#define BOARD_UART_IN_DMA_BUFFER_SIZE 8192u
#define BOARD_UART_IN_PROCESS_BYTE_BUDGET 512u
#define BOARD_SAMPLE_PERIOD_MS 10u

#if (BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND != 0u)
#define BOARD_TIM2_UPDATE_HZ 1000u
#define BOARD_TIM2_TICK_MS 1u
#if (BOARD_TIM2_FIXED_SEND_DIVIDER == 0u)
#error "BOARD_TIM2_FIXED_SEND_DIVIDER must be >= 1"
#endif
#define BOARD_TIM2_FIXED_SEND_HZ (BOARD_TIM2_UPDATE_HZ / BOARD_TIM2_FIXED_SEND_DIVIDER)
#else
#define BOARD_TIM2_UPDATE_HZ 500u
#define BOARD_TIM2_TICK_MS 2u
#define BOARD_TIM2_FIXED_SEND_HZ (1000u / BOARD_SAMPLE_PERIOD_MS)
#endif

/* Last 8-Kbyte H503 sector. The linker script excludes this sector. */
#define BOARD_FLASH_CONFIG_ADDRESS 0x0801E000u
#define BOARD_FLASH_CONFIG_SIZE 0x2000u

/** @brief 三颗 MLX90393 的 7 位 I2C 地址表，数组顺序同时表达板级角色。 */
extern const uint8_t g_board_mlx_addresses[BOARD_MLX_COUNT];
#if (BOARD_IMU_COUNT > 0u)
/** @brief 两颗 LSM6DSOW 的 7 位 I2C 地址表；指腹磁铁方案没有 IMU，因此不声明该地址表。 */
extern const uint8_t g_board_imu_addresses[BOARD_IMU_COUNT];
#endif

#endif
