/**
 * @file bsp_uart_dma.c
 * @brief LPUART1 DMA 异步发送及缓冲区占用保护。
 * @details 所属层级：BSP。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#include "bsp_uart_dma.h"

#include "bsp_handles.h"

static volatile uint8_t g_tx_busy;

/**
 * @brief 检查参数和 DMA 状态后启动 LPUART1 异步发送。
 * @details 启动前先置忙，HAL 启动失败时立即回滚，避免并发覆盖发送缓冲区。
 */
AppStatus BspUartDma_Transmit(const uint8_t *data, uint16_t length) {
  HAL_StatusTypeDef result;
  if ((data == NULL) || (length == 0u)) return APP_STATUS_BAD_PARAM;
  if (g_tx_busy != 0u) return APP_STATUS_BUSY;
  if (hlpuart1.hdmatx == NULL) return APP_STATUS_NOT_READY;
  /* 必须在调用 HAL 前置忙，防止极短 DMA 完成中断与主循环产生竞争。 */
  g_tx_busy = 1u;
  /* HAL 只保存缓冲区指针，调用者必须在完成或错误回调前保持数据不变。 */
  result = HAL_UART_Transmit_DMA(&hlpuart1, data, length);
  if (result != HAL_OK) {
    g_tx_busy = 0u;
    return (result == HAL_BUSY) ? APP_STATUS_BUSY : APP_STATUS_IO;
  }
  return APP_STATUS_OK;
}

/** @brief DMA 正常完成后的状态收尾。 */
void BspUartDma_TxComplete(void) {
  g_tx_busy = 0u;
}

/** @brief DMA 或 UART 错误后的状态恢复。 */
void BspUartDma_TxError(void) {
  g_tx_busy = 0u;
}

/** @brief 返回软件维护的 TX 占用标志。 */
uint8_t BspUartDma_IsBusy(void) {
  return g_tx_busy;
}




