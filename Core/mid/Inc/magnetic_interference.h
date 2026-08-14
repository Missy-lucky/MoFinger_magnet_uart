/**
 * @file magnetic_interference.h
 * @brief 双参考 MLX 磁干扰诊断结果、标志和接口。
 * @details 所属层级：MID。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef MAGNETIC_INTERFERENCE_H
#define MAGNETIC_INTERFERENCE_H

#include <stdint.h>
#include "mlx90393.h"

/** @brief 参考磁传感器诊断标志：参考探头 1 的场强超过暂定阈值。 */
#define MAG_INTERFERENCE_REF1_HIGH (1u << 0)
/** @brief 参考磁传感器诊断标志：参考探头 2 的场强超过暂定阈值。 */
#define MAG_INTERFERENCE_REF2_HIGH (1u << 1)
/** @brief 参考磁传感器诊断标志：两颗参考探头的场强差超过暂定阈值。 */
#define MAG_INTERFERENCE_REFERENCE_MISMATCH (1u << 2)
/** @brief 参考磁传感器诊断标志：本周期缺少至少一颗参考探头的有效数据。 */
#define MAG_INTERFERENCE_DATA_INVALID (1u << 7)

/** @brief 两颗参考 MLX90393 计算得到的磁干扰诊断结果。 */
typedef struct {
  float reference_1_ut;
  float reference_2_ut;
  float reference_difference_ut;
  uint8_t flags;
} MagneticInterferenceResult;

/**
 * @brief 根据两颗磁干扰参考传感器的原始三轴数据生成诊断结果。
 * @param reference_1 U6（地址 0x0D）的参考传感器样本。
 * @param reference_2 U8（地址 0x0E）的参考传感器样本。
 * @param sensitivity 当前 MLX 配置对应的三轴灵敏度。
 * @param reference_valid_mask 位 0/1 分别表示两个输入样本是否有效。
 * @param result 输出的场强和诊断标志。
 * @details 三轴按当前 MLX90393 灵敏度换算为 uT 后计算向量模长。阈值只是未标定阶段的
 *          工程初值，最终应根据整机无干扰数据和目标干扰源实验重新设置。
 */
void MagneticInterference_Evaluate(const Mlx90393Sample *reference_1,
                                    const Mlx90393Sample *reference_2,
                                    const Mlx90393Sensitivity *sensitivity,
                                    uint8_t reference_valid_mask,
                                    MagneticInterferenceResult *result);

#endif

