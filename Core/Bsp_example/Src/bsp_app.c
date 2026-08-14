#include "bsp_app.h"

#include "app_config.h"
#include "main.h"
#include "sensor_topology.h"
#include "soft_i2c.h"

/* 板级适配层：
 * 这里处理 LED 心跳、调试串口句柄导出，以及软 I2C 总线的一次性初始化。
 */

extern UART_HandleTypeDef huart1;

static uint32_t g_led_last_tick;

/* 初始化板级自定义外设。 */
void BspApp_Init(void)
{
  GPIO_InitTypeDef led_gpio = {0};

  /* 心跳 LED 由应用层软件控制，因此这里手工初始化，而不是依赖 Cube 自动生成。 */
  led_gpio.Pin = APP_LED_Pin;
  led_gpio.Mode = GPIO_MODE_OUTPUT_PP;
  led_gpio.Pull = GPIO_NOPULL;
  led_gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(APP_LED_GPIO_Port, &led_gpio);
  /* 上电默认灭灯，直到 heartbeat 周期开始后再闪烁。 */
  HAL_GPIO_WritePin(APP_LED_GPIO_Port, APP_LED_Pin, GPIO_PIN_RESET);
  g_led_last_tick = HAL_GetTick();

  /* 软 I2C 是后续 boot 初始化和运行期采样的基础，因此在最早阶段完成。 */
  (void)soft_i2c_init_all(g_bus_pins, SENSOR_BUS_COUNT, APP_I2C_DELAY_US, APP_I2C_ACK_TIMEOUT_US);
}

/* 心跳 LED 周期翻转。 */
void BspApp_ServiceHeartbeat(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - g_led_last_tick) >= APP_LED_HEARTBEAT_MS)
  {
    /* 每到一个心跳周期就翻转一次，形成“系统活着”的低成本指示。 */
    g_led_last_tick = now;
    HAL_GPIO_TogglePin(APP_LED_GPIO_Port, APP_LED_Pin);
  }
}

/* 导出项目当前使用的调试串口句柄。 */
UART_HandleTypeDef *BspApp_GetConsoleUart(void)
{
  return &huart1;
}
