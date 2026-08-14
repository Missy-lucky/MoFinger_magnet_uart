#ifndef BSP_APP_H
#define BSP_APP_H

#include "stm32f4xx_hal.h"

/* BSP 适配层：
 * 把“这块板子的具体外设实例和引脚”封装成统一接口，避免上层业务直接依赖 Cube 生成符号。
 */
/* 初始化板级自定义外设：
 * 当前主要包括心跳 LED 初始状态，以及整板软 I2C 总线初始化。
 */
void BspApp_Init(void);
/* 周期服务心跳 LED。
 * 设计成无状态外部调用接口，谁在主循环里方便，谁就定期调用它。
 */
void BspApp_ServiceHeartbeat(void);
/* 返回调试控制台当前绑定的 UART 实例。
 * 之所以不直接暴露全局 huart1，是为了给后续换串口或多板适配留出空间。
 */
UART_HandleTypeDef *BspApp_GetConsoleUart(void);

#endif /* BSP_APP_H */
