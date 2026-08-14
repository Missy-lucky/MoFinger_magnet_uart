/**
 * @file sensor_config.h
 * @brief 应用层传感器运行参数声明。
 * @details 所属层级：APP。项目调参集中在本头文件的 #define：修改增益、Hall spinning、
 *          分辨率、滤波、过采样（MLX90393）以及量程、输出速率（LSM6DSOW）时，只编辑
 *          本文件对应的宏，无需改动 sensor_config.c、驱动层或任何子对象。
 */

#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include "mlx90393.h"
#include "lsm6dsow.h"

/* ==========================================================================
 * MLX90393 采样配置（三颗共用）
 * 每个宏对应芯片寄存器位域，改这里即可，sensor_config.c 只做引用。
 * ========================================================================== */
#define SENSOR_MLX_GAIN_SEL 5u    /* GAIN_SEL：0..7，增益选择 */
#define SENSOR_MLX_HALLCONF 0x0Cu /* HALLCONF：Hall spinning 相位，典型 0xC */
#define SENSOR_MLX_RES_X    1u    /* RES_X：X 轴分辨率 0..3 */
#define SENSOR_MLX_RES_Y    1u    /* RES_Y：Y 轴分辨率 0..3 */
#define SENSOR_MLX_RES_Z    1u    /* RES_Z：Z 轴分辨率 0..3 */
#define SENSOR_MLX_DIG_FILT 2u    /* DIG_FILT：数字滤波 0..7 */
#define SENSOR_MLX_OSR      0u    /* OSR：磁场过采样 0..3 */
#define SENSOR_MLX_OSR2     0u    /* OSR2：温度过采样 0..3 */
#define SENSOR_MLX_TCMP_EN  0u    /* TCMP_EN：温度补偿 0=关 1=开 */
#define SENSOR_MLX_BURST_SEL 0x0Fu /* BURST_SEL/命令 zyxt：温度和 XYZ 全部转换 */
#define SENSOR_MLX_BURST_DATA_RATE 0u /* BDR：0=连续 burst；非 0 为 BDR*20ms 间隔 */

/* ==========================================================================
 * LSM6DSOW 采样配置（两颗共用）
 * 直接写寄存器语义值；三个 CTRL 寄存器的最终字节由下面的宏组合而成。
 * 常用取值：
 *   ODR   0x40=104Hz 0x50=208Hz 0x60=416Hz
 *   FS_XL 0x00=±2g   0x08=±16g  0x0C=±8g  0x04=±4g
 *   FS_G  0x00=±250  0x04=±500  0x08=±1000 0x0C=±2000 dps
 * ========================================================================== */
#define SENSOR_IMU_XL_ODR   0x40u /* 加速度 ODR：0x40=104 Hz */
#define SENSOR_IMU_XL_FS    0x08u /* 加速度量程：0x08=±16 g */
#define SENSOR_IMU_G_ODR    0x40u /* 陀螺 ODR：0x40=104 Hz */
#define SENSOR_IMU_G_FS     0x0Cu /* 陀螺量程：0x0C=±2000 dps */
/* CTRL3_C：BDU（高低字节一致）+ IF_INC（寄存器自增，支持连续读）。 */
#define SENSOR_IMU_CTRL3_C  0x44u

/* 由上面语义宏组合出实际写入寄存器的字节，驱动层直接取用。 */
#define SENSOR_IMU_CTRL1_XL (SENSOR_IMU_XL_ODR | SENSOR_IMU_XL_FS)
#define SENSOR_IMU_CTRL2_G  (SENSOR_IMU_G_ODR | SENSOR_IMU_G_FS)

/** @brief 三颗 MLX90393 当前共用的项目默认配置（值来自上方 #define）。 */
extern const Mlx90393Config g_sensor_mlx_config;

/** @brief 两颗 LSM6DSOW 当前共用的项目默认配置（值来自上方 #define）。 */
extern const Lsm6dsowConfig g_sensor_imu_config;

#endif
