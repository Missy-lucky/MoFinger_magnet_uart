/**
 * @file orientation.h
 * @brief 六轴 IMU 互补滤波姿态状态和接口。
 * @details 所属层级：MID。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdint.h>
#include "lsm6dsow.h"

/** @brief 单颗 IMU 的欧拉角估算状态，角度单位均为度。 */
typedef struct {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  uint8_t initialized;
} OrientationState;

/**
 * @brief 清零姿态状态并标记为尚未用加速度初始化。
 * @param state 待重置状态；NULL 时不执行操作。
 */
void Orientation_Reset(OrientationState *state);

/**
 * @brief 使用一帧六轴数据更新互补滤波姿态。
 * @param state 持久化姿态状态。
 * @param sample LSM6DSOW 原始六轴样本。
 * @param dt_seconds 相邻有效样本的时间间隔，单位秒，必须大于 0。
 * @details roll/pitch 由陀螺积分和重力方向融合；yaw 仅陀螺积分，会随时间漂移。
 */
void Orientation_Update(OrientationState *state, const Lsm6dsowSample *sample, float dt_seconds);

#endif

