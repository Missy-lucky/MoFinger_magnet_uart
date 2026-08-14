/**
 * @file bsp_time.h
 * @brief 板级毫秒计时和初始化延时接口。
 * @details 所属层级：BSP。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

#include "app_status.h"

/**
 * @brief 启动 TIM2 更新中断并清零板级毫秒计数。
 * @return 启动成功返回 APP_STATUS_OK，否则返回 APP_STATUS_IO。
 * @note 必须在 CubeMX 完成 MX_TIM2_Init() 后、使用其他时间接口前调用。
 */
AppStatus BspTime_Init(void);

/**
 * @brief 获取系统启动后的毫秒计数。
 * @return 32 位毫秒 tick，按无符号整数规则自然回绕。
 */
uint32_t BspTime_GetTickMs(void);

/**
 * @brief 阻塞延时指定毫秒数。
 * @param delay_ms 延时时长，单位毫秒。
 * @note 仅用于器件初始化等非实时路径，周期采集不得依赖该阻塞延时。
 */
void BspTime_DelayMs(uint32_t delay_ms);

/**
 * @brief 消耗一个由 TIM2 更新中断产生的固定发送节拍。
 * @return 有待处理节拍返回 1，否则返回 0。
 * @note 1000 Hz 固定发帧分支使用该接口把中断事件转移到主循环执行。
 */
uint8_t BspTime_ConsumeFrameTick(void);

/**
 * @brief 清空初始化期间累积的固定发送节拍。
 * @return 无返回值。
 */
void BspTime_ResetFrameTick(void);

/**
 * @brief 处理 TIM2 周期到期事件，将板级毫秒计数增加当前配置的 tick 步长。
 * @return 无返回值。
 * @note 只允许由 HAL_TIM_PeriodElapsedCallback() 在确认 htim2 后调用。
 */
void BspTime_OnTim2Elapsed(void);

#endif

