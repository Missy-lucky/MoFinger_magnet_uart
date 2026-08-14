/**
 * @file bsp_time.c
 * @brief 基于 TIM2 更新中断的板级毫秒时间基准、固定发帧节拍和阻塞延时。
 * @details 所属层级：BSP。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "bsp_time.h"

#include "board_config.h"
#include "bsp_handles.h"

/** @brief TIM2 更新中断推进的 32 位毫秒计数。 */
static volatile uint32_t g_tim2_tick_ms;
/** @brief TIM2 更新中断产生、由主循环消费的固定发帧节拍。 */
static volatile uint8_t g_frame_tick_pending;

/** @brief 清零板级时间并启动 TIM2 更新中断。 */
AppStatus BspTime_Init(void) {
  /* 先清除软件计数和硬件计数，保证每次初始化都从确定的零点开始。 */
  g_tim2_tick_ms = 0u;
  g_frame_tick_pending = 0u;
  __HAL_TIM_SET_COUNTER(&htim2, 0u);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  /* TIM2 计数频率为 100 kHz；ARR 由编译宏选择 500 Hz 或 1000 Hz 更新。 */
  return (HAL_TIM_Base_Start_IT(&htim2) == HAL_OK) ? APP_STATUS_OK
                                                   : APP_STATUS_IO;
}

/** @brief 返回由 TIM2 更新中断维护的板级毫秒 tick。 */
uint32_t BspTime_GetTickMs(void) {
  /* Cortex-M33 对齐 32 位读写为原子操作，上层按无符号差处理自然回绕。 */
  return g_tim2_tick_ms;
}

/** @brief 基于 TIM2 毫秒 tick 实现初始化阶段的阻塞延时。 */
void BspTime_DelayMs(uint32_t delay_ms) {
  uint32_t started = BspTime_GetTickMs();

  /* 使用差值比较兼容 32 位计数回绕；周期采集路径不调用该阻塞接口。 */
  while ((BspTime_GetTickMs() - started) < delay_ms) {
  }
}

/** @brief 消耗一个 TIM2 固定发帧节拍；主循环落后时不补发历史 tick。 */
uint8_t BspTime_ConsumeFrameTick(void) {
  if (g_frame_tick_pending == 0u) return 0u;
  __disable_irq();
  if (g_frame_tick_pending != 0u) {
    g_frame_tick_pending = 0u;
    __enable_irq();
    return 1u;
  }
  __enable_irq();
  return 0u;
}

/** @brief 清空初始化阶段累积的发帧节拍，避免启动后补发旧 tick。 */
void BspTime_ResetFrameTick(void) {
  __disable_irq();
  g_frame_tick_pending = 0u;
  __enable_irq();
}

/** @brief 在 TIM2 每次更新时推进板级毫秒计数并记录一个固定发帧节拍。 */
void BspTime_OnTim2Elapsed(void) {
  g_tim2_tick_ms += BOARD_TIM2_TICK_MS;
  g_frame_tick_pending = 1u;
}



