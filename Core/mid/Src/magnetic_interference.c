/**
 * @file magnetic_interference.c
 * @brief 参考磁场模长、差值和暂定阈值诊断实现。
 * @details 所属层级：MID。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "magnetic_interference.h"

#include <stddef.h>

/* 未完成整机标定前使用的保守工程初值，单位均为 uT。 */
/* 两个阈值为整机标定前的占位值，量产前必须用实测分布替换。 */
#define MAG_REFERENCE_HIGH_THRESHOLD_UT 500.0f
#define MAG_REFERENCE_MISMATCH_THRESHOLD_UT 250.0f

/**
 * @brief 使用固定次数牛顿迭代计算非负浮点数平方根。
 * @param value 非负输入值。
 * @return 平方根近似值；输入小于等于零时返回零。
 */
static float mag_sqrt(float value) {
  float estimate;
  if (value <= 0.0f) return 0.0f;
  estimate = (value > 1.0f) ? value : 1.0f;
  for (uint8_t i = 0u; i < 20u; ++i) estimate = 0.5f * (estimate + value / estimate);
  return estimate;
}

/**
 * @brief 把 MLX90393 原始 XYZ 换算为磁场向量模长。
 * @param sample 输入的原始磁传感器样本。
 * @param sensitivity 当前 MLX 配置对应的三轴灵敏度。
 * @return 使用调用者提供的灵敏度换算后的磁场模长，单位 uT。
 */
static float field_magnitude_ut(const Mlx90393Sample *sample,
                                const Mlx90393Sensitivity *sensitivity) {
  /* X/Y 与 Z 的标称灵敏度不同，必须分别换算后再求模长。 */
  float x = (float)sample->x * sensitivity->x_ut_per_lsb;
  float y = (float)sample->y * sensitivity->y_ut_per_lsb;
  float z = (float)sample->z * sensitivity->z_ut_per_lsb;
  return mag_sqrt((x * x) + (y * y) + (z * z));
}

/** @brief 实现双参考探头场强换算、有效性检查和暂定阈值判断。 */
void MagneticInterference_Evaluate(const Mlx90393Sample *reference_1,
                                    const Mlx90393Sample *reference_2,
                                    const Mlx90393Sensitivity *sensitivity,
                                    uint8_t reference_valid_mask,
                                    MagneticInterferenceResult *result) {
  float difference;
  if (result == NULL) return;

  /* 每次先清零完整输出，错误路径不会泄漏上一周期诊断值。 */
  result->reference_1_ut = 0.0f;
  result->reference_2_ut = 0.0f;
  result->reference_difference_ut = 0.0f;
  result->flags = 0u;
  /* 两颗参考探头必须同时有效，否则只报告数据无效，不进行阈值判断。 */
  if ((reference_1 == NULL) || (reference_2 == NULL) || (sensitivity == NULL) ||
      ((reference_valid_mask & 0x03u) != 0x03u)) {
    result->flags = MAG_INTERFERENCE_DATA_INVALID;
    return;
  }

  /* 数据完整后计算两个模长及绝对差，再分别生成可组合的诊断位。 */
  result->reference_1_ut = field_magnitude_ut(reference_1, sensitivity);
  result->reference_2_ut = field_magnitude_ut(reference_2, sensitivity);
  difference = result->reference_1_ut - result->reference_2_ut;
  if (difference < 0.0f) difference = -difference;
  result->reference_difference_ut = difference;

  if (result->reference_1_ut >= MAG_REFERENCE_HIGH_THRESHOLD_UT) {
    result->flags |= MAG_INTERFERENCE_REF1_HIGH;
  }
  if (result->reference_2_ut >= MAG_REFERENCE_HIGH_THRESHOLD_UT) {
    result->flags |= MAG_INTERFERENCE_REF2_HIGH;
  }
  if (difference >= MAG_REFERENCE_MISMATCH_THRESHOLD_UT) {
    result->flags |= MAG_INTERFERENCE_REFERENCE_MISMATCH;
  }
}






