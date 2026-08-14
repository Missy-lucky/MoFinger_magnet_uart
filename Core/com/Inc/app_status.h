/**
 * @file app_status.h
 * @brief 公共层统一状态码定义，隔离上层代码与 STM32 HAL 返回类型。
 * @details 所属层级：COM。本文档注释说明文件职责、接口约束和关键实现步骤。
 */

#ifndef APP_STATUS_H
#define APP_STATUS_H

/** @brief 项目各层统一使用的返回状态。 */
typedef enum {
  APP_STATUS_OK = 0,
  APP_STATUS_BUSY,
  APP_STATUS_TIMEOUT,
  APP_STATUS_IO,
  APP_STATUS_BAD_PARAM,
  APP_STATUS_BAD_ID,
  APP_STATUS_VERIFY,
  APP_STATUS_NOT_READY
} AppStatus;

#endif

