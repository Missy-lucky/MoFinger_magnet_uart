/**
 * @file bsp_uart_in.c
 * @brief USART1 IN port DMA receive buffer.
 * @details BOARD_ENABLE_USART1_IN=1 starts USART1 RX by DMA. The app consumes
 *          bytes from a large DMA-backed ring view, not from per-byte IRQs.
 */

#include "bsp_uart_in.h"

#include "bsp_handles.h"

#if (BOARD_ENABLE_USART1_IN != 0u)

#if (BOARD_UART_IN_DMA_BUFFER_SIZE < 1024u)
#error "BOARD_UART_IN_DMA_BUFFER_SIZE must be at least 1024"
#endif

static uint8_t g_rx_dma_buffer[BOARD_UART_IN_DMA_BUFFER_SIZE];
static volatile uint32_t g_rx_write_total;
static volatile uint32_t g_rx_read_total;
static volatile uint16_t g_rx_dma_position;
static volatile uint32_t g_rx_overflow_count;
static volatile uint8_t g_rx_dma_running;

static uint16_t dma_write_position(void) {
  uint32_t remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
  if (remaining > BOARD_UART_IN_DMA_BUFFER_SIZE) {
    remaining = BOARD_UART_IN_DMA_BUFFER_SIZE;
  }
  return (uint16_t)(BOARD_UART_IN_DMA_BUFFER_SIZE - remaining);
}

static void update_write_total_from_dma(void) {
  uint16_t position;
  uint16_t previous;
  uint32_t delta;
  uint32_t capacity = BOARD_UART_IN_DMA_BUFFER_SIZE - 1u;

  if ((g_rx_dma_running == 0u) || (huart1.hdmarx == NULL)) return;

  position = dma_write_position();
  previous = g_rx_dma_position;
  if (position >= previous) {
    delta = (uint32_t)(position - previous);
  } else {
    delta = (uint32_t)(BOARD_UART_IN_DMA_BUFFER_SIZE - previous) + position;
  }

  if (delta != 0u) {
    g_rx_write_total += delta;
    g_rx_dma_position = position;
  }

  if ((g_rx_write_total - g_rx_read_total) > capacity) {
    uint32_t lost = (g_rx_write_total - g_rx_read_total) - capacity;
    g_rx_read_total = g_rx_write_total - capacity;
    g_rx_overflow_count += lost;
  }
}

static AppStatus start_receive(void) {
  HAL_StatusTypeDef result;

  if (huart1.hdmarx == NULL) return APP_STATUS_NOT_READY;

  result = HAL_UART_Receive_DMA(&huart1, g_rx_dma_buffer,
                                BOARD_UART_IN_DMA_BUFFER_SIZE);
  if (result == HAL_OK) {
    g_rx_dma_running = 1u;
    g_rx_dma_position = 0u;
    return APP_STATUS_OK;
  }
  return (result == HAL_BUSY) ? APP_STATUS_BUSY : APP_STATUS_IO;
}

AppStatus BspUartIn_Init(void) {
  g_rx_write_total = 0u;
  g_rx_read_total = 0u;
  g_rx_dma_position = 0u;
  g_rx_overflow_count = 0u;
  g_rx_dma_running = 0u;
  return start_receive();
}

uint8_t BspUartIn_ReadByte(uint8_t *byte) {
  uint32_t read_total;

  if (byte == NULL) return 0u;

  __disable_irq();
  update_write_total_from_dma();
  read_total = g_rx_read_total;
  if (read_total == g_rx_write_total) {
    __enable_irq();
    return 0u;
  }
  *byte = g_rx_dma_buffer[read_total % BOARD_UART_IN_DMA_BUFFER_SIZE];
  g_rx_read_total = read_total + 1u;
  __enable_irq();

  return 1u;
}

void BspUartIn_OnRxComplete(void) {
  update_write_total_from_dma();
  g_rx_dma_running = 0u;
  (void)start_receive();
}

void BspUartIn_OnError(void) {
  g_rx_dma_running = 0u;
  (void)HAL_UART_AbortReceive(&huart1);
  (void)start_receive();
}

#else

AppStatus BspUartIn_Init(void) {
  return APP_STATUS_OK;
}

uint8_t BspUartIn_ReadByte(uint8_t *byte) {
  (void)byte;
  return 0u;
}

void BspUartIn_OnRxComplete(void) {
}

void BspUartIn_OnError(void) {
}

#endif
