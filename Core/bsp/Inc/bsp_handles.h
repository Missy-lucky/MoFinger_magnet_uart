/**
 * @file bsp_handles.h
 * @brief 声明 CubeMX 集中式 main.c 中创建的外设句柄。
 * @details 所属层级：BSP。驱动和应用不得直接包含或访问这些 HAL 句柄。
 */

#ifndef BSP_HANDLES_H
#define BSP_HANDLES_H

#include "stm32h5xx_hal.h"

/** @brief I2C1 HAL 句柄，由 CubeMX 生成的 main.c 定义。 */
extern I2C_HandleTypeDef hi2c1;
/** @brief LPUART1 HAL 句柄，由 CubeMX 生成的 main.c 定义。 */
extern UART_HandleTypeDef hlpuart1;
/** @brief USART1 HAL 句柄；指腹磁铁方案作为 IN 口使用。 */
extern UART_HandleTypeDef huart1;
/** @brief LPUART1 TX 使用的 GPDMA1 Channel 0 句柄。 */
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
/** @brief USART1 RX 使用的 GPDMA1 Channel 1 句柄。 */
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
/** @brief 1 ms 板级时间基准使用的 TIM2 HAL 句柄。 */
extern TIM_HandleTypeDef htim2;

#endif
