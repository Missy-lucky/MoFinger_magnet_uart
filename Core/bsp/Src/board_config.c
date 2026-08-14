/**
 * @file board_config.c
 * @brief 原理图确定的传感器地址表定义。
 * @details 所属层级：BSP。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "board_config.h"

/** @brief 三颗 MLX90393 的板级 7 位地址表，顺序为主传感器、参考 1、参考 2。 */
/* 数组索引同时表达业务角色：0 为主探头，1、2 为磁干扰参考探头。 */
const uint8_t g_board_mlx_addresses[BOARD_MLX_COUNT] = {
    BOARD_MLX_PRIMARY_ADDR,
    BOARD_MLX_INTERFERENCE_1_ADDR,
    BOARD_MLX_INTERFERENCE_2_ADDR,
};

#if (BOARD_IMU_COUNT > 0u)
/** @brief 两颗 LSM6DSOW 的板级 7 位地址表；指尖板硬件保留 IMU，但当前默认不写入上报帧。 */
/* 两颗 IMU 的顺序与协议和姿态状态数组保持一致。 */
const uint8_t g_board_imu_addresses[BOARD_IMU_COUNT] = {
    BOARD_IMU_1_ADDR,
    BOARD_IMU_2_ADDR,
};
#endif


