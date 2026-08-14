/**
 * @file sensor_app.h
 * @brief 原始传感器采集、DMA 上报和 UART 回调入口。
 * @details 所属层级：APP。接口不暴露姿态或磁干扰算法状态。
 */

#ifndef SENSOR_APP_H
#define SENSOR_APP_H

/**
 * @brief 初始化三颗 MLX90393、两颗 LSM6DSOW 和采样调度器。
 * @note 必须在 CubeMX 外设初始化完成后调用。
 */
void SensorApp_Init(void);

/**
 * @brief 在主循环中反复调用，按固定周期采集并发送原始传感器数据。
 * @note 函数不会等待 UART DMA 完成，也不执行姿态识别或磁干扰判断。
 */
void SensorApp_Process(void);

/** @brief 由 HAL UART 发送完成回调调用，释放 DMA 发送占用状态。 */
void SensorApp_OnUartTxComplete(void);

/** @brief 由 HAL UART 错误回调调用，恢复 DMA 发送状态。 */
void SensorApp_OnUartError(void);

#endif
