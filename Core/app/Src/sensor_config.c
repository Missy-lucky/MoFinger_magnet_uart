/**
 * @file sensor_config.c
 * @brief 灵巧手指尖传感器项目参数定义。
 * @details 所属层级：APP。本文件不再包含任何字面量，全部字段引用 sensor_config.h
 *          中的 #define；调参时只编辑那些宏，本文件无需改动。
 */

#include "sensor_config.h"

/**
 * @brief 三颗 MLX90393 共用的运行配置，字段值全部来自 sensor_config.h 的宏。
 */
const Mlx90393Config g_sensor_mlx_config = {
    .gain_sel = SENSOR_MLX_GAIN_SEL,
    .hallconf = SENSOR_MLX_HALLCONF,
    .res_x = SENSOR_MLX_RES_X,
    .res_y = SENSOR_MLX_RES_Y,
    .res_z = SENSOR_MLX_RES_Z,
    .dig_filt = SENSOR_MLX_DIG_FILT,
    .osr = SENSOR_MLX_OSR,
    .osr2 = SENSOR_MLX_OSR2,
    .tcmp_en = SENSOR_MLX_TCMP_EN,
    .burst_sel = SENSOR_MLX_BURST_SEL,
    .burst_data_rate = SENSOR_MLX_BURST_DATA_RATE,
};

/**
 * @brief 两颗 LSM6DSOW 共用的运行配置，字段值全部来自 sensor_config.h 的宏。
 * @details 三个字段直接就是 CTRL1_XL / CTRL2_G / CTRL3_C 要写入的寄存器字节。
 */
const Lsm6dsowConfig g_sensor_imu_config = {
    .ctrl1_xl = SENSOR_IMU_CTRL1_XL,
    .ctrl2_g = SENSOR_IMU_CTRL2_G,
    .ctrl3_c = SENSOR_IMU_CTRL3_C,
};
