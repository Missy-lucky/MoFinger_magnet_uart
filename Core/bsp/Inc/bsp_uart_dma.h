/**
 * @file bsp_uart_dma.h
 * @brief LPUART1 TX DMA 生命周期接口。
 * @details 所属层级：BSP。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef BSP_UART_DMA_H
#define BSP_UART_DMA_H

#include <stdint.h>
#include "app_status.h"

/**
 * @brief 使用 LPUART1 TX DMA 异步发送一段连续数据。
 * @param data 发送缓冲区；DMA 完成前内容必须保持不变。
 * @param length 发送字节数。
 * @return APP_STATUS_OK 表示 DMA 已启动；BUSY 表示上一帧未完成；其余为参数或 HAL 错误。
 */
AppStatus BspUartDma_Transmit(const uint8_t *data, uint16_t length);

/** @brief 在 HAL UART TX 完成回调中调用，清除软件忙标志。 */
void BspUartDma_TxComplete(void);

/** @brief 在 HAL UART 错误回调中调用，释放软件忙标志以允许恢复发送。 */
void BspUartDma_TxError(void);

/** @brief 查询当前是否存在尚未完成的 DMA 发送。 */
uint8_t BspUartDma_IsBusy(void);

#endif

