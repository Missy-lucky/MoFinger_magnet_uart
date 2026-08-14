/**
 * @file bsp_uart_in.h
 * @brief USART1 IN port receive entry.
 * @details Finger-pulp firmware uses USART1 as IN port; fingertip firmware
 *          keeps empty implementations.
 */

#ifndef BSP_UART_IN_H
#define BSP_UART_IN_H

#include <stdint.h>

#include "app_status.h"
#include "board_config.h"

/** @brief Start USART1 IN RX DMA. */
AppStatus BspUartIn_Init(void);

/**
 * @brief Read one byte from the USART1 IN DMA-backed buffer.
 * @param byte Output byte pointer.
 * @return 1 when one byte was read, 0 when no data is available.
 */
uint8_t BspUartIn_ReadByte(uint8_t *byte);

/** @brief USART1 RX DMA block-complete callback entry; restarts the next block. */
void BspUartIn_OnRxComplete(void);

/** @brief USART1 receive error callback entry; aborts and restarts RX DMA. */
void BspUartIn_OnError(void);

#endif
