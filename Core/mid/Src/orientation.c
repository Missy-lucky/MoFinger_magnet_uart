/**
 * @file orientation.c
 * @brief 无 libm 的 roll、pitch、yaw 姿态估算实现。
 * @details 所属层级：MID。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "orientation.h"

#include <stddef.h>

/* 算法常量与当前陀螺量程固定对应；更改驱动量程时必须同步更新。 */
#define RAD_TO_DEG 57.2957795f
#define GYRO_2000DPS_MDPS_PER_LSB 70.0f
#define COMPLEMENTARY_GYRO_WEIGHT 0.98f
#define PI 3.14159265f

/**
 * @brief 返回浮点数绝对值，避免引入 libm。
 * @param value 输入值。
 * @return value 的非负幅值。
 */
static float fast_abs(float value) {
  return (value < 0.0f) ? -value : value;
}

/**
 * @brief 使用二十次牛顿迭代近似平方根。
 * @param value 非负输入值。
 * @return 平方根近似值；输入小于等于 0 时返回 0。
 * @details 迭代次数覆盖 int16 三轴平方和范围，不依赖 libm。
 */
static float fast_sqrt(float value) {
  float estimate;
  if (value <= 0.0f) return 0.0f;
  estimate = (value > 1.0f) ? value : 1.0f;
  for (uint8_t i = 0u; i < 20u; ++i) {
    estimate = 0.5f * (estimate + value / estimate);
  }
  return estimate;
}

/**
 * @brief 使用低成本近似计算 atan2。
 * @param y 向量 Y 分量。
 * @param x 向量 X 分量。
 * @return 位于约 [-pi, pi] 的弧度角。
 * @details 该近似仅用于互补滤波的低带宽重力校正，不用于高精度测量。
 */
static float fast_atan2(float y, float x) {
  const float quarter_pi = PI * 0.25f;
  float abs_y = fast_abs(y) + 1.0e-10f;
  float angle;
  float ratio;
  if (x >= 0.0f) {
    ratio = (x - abs_y) / (x + abs_y);
    angle = quarter_pi - quarter_pi * ratio;
  } else {
    ratio = (x + abs_y) / (abs_y - x);
    angle = 3.0f * quarter_pi - quarter_pi * ratio;
  }
  return (y < 0.0f) ? -angle : angle;
}

/** @brief 实现姿态状态复位。 */
void Orientation_Reset(OrientationState *state) {
  if (state == NULL) return;
  state->roll_deg = 0.0f;
  state->pitch_deg = 0.0f;
  state->yaw_deg = 0.0f;
  state->initialized = 0u;
}

/** @brief 实现陀螺预测与加速度校正的互补滤波。 */
void Orientation_Update(OrientationState *state, const Lsm6dsowSample *sample, float dt_seconds) {
  float ax;
  float ay;
  float az;
  float accel_roll;
  float accel_pitch;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  if ((state == NULL) || (sample == NULL) || (dt_seconds <= 0.0f)) return;

  /* 加速度原始比例在 atan2 比值中抵消，无需先换算为 g。 */
  ax = (float)sample->accel[0];
  ay = (float)sample->accel[1];
  az = (float)sample->accel[2];
  accel_roll = fast_atan2(ay, az) * RAD_TO_DEG;
  accel_pitch = fast_atan2(-ax, fast_sqrt((ay * ay) + (az * az))) * RAD_TO_DEG;

  /* 按正负 2000 dps 档的 70 mdps/LSB 将陀螺原始量换算为角速度。 */
  gx_dps = (float)sample->gyro[0] * GYRO_2000DPS_MDPS_PER_LSB * 0.001f;
  gy_dps = (float)sample->gyro[1] * GYRO_2000DPS_MDPS_PER_LSB * 0.001f;
  gz_dps = (float)sample->gyro[2] * GYRO_2000DPS_MDPS_PER_LSB * 0.001f;

  /* 首帧直接用重力方向初始化 roll/pitch，避免从零度缓慢收敛。 */
  if (state->initialized == 0u) {
    state->roll_deg = accel_roll;
    state->pitch_deg = accel_pitch;
    state->yaw_deg = 0.0f;
    state->initialized = 1u;
    return;
  }

  /* 高频运动跟随陀螺积分，低频漂移由加速度重力方向校正。 */
  state->roll_deg = COMPLEMENTARY_GYRO_WEIGHT * (state->roll_deg + gx_dps * dt_seconds) +
                    (1.0f - COMPLEMENTARY_GYRO_WEIGHT) * accel_roll;
  state->pitch_deg = COMPLEMENTARY_GYRO_WEIGHT * (state->pitch_deg + gy_dps * dt_seconds) +
                     (1.0f - COMPLEMENTARY_GYRO_WEIGHT) * accel_pitch;
  /* 没有可信航向参考时 yaw 只能积分，长期漂移属于算法已知边界。 */
  state->yaw_deg += gz_dps * dt_seconds;
}







