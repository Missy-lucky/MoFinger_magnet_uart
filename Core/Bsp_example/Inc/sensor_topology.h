#ifndef SENSOR_TOPOLOGY_H
#define SENSOR_TOPOLOGY_H

#include <stdint.h>
#include "main.h"
#include "app_config.h"

/* 一条软 I2C 总线的物理引脚绑定。 */
typedef struct {
  GPIO_TypeDef *scl_port;
  uint16_t scl_pin;
  GPIO_TypeDef *sda_port;
  uint16_t sda_pin;
} SoftI2CBusPins;

/* 一颗在某条总线上的轮询传感器定义。 */
typedef struct {
  const char *name;
  uint8_t addr7;
} PollSensor;

/* 自定义轮询清单使用的扁平化定义。 */
typedef struct {
  const char *name;
  uint8_t bus;
  uint8_t slot;
  uint8_t addr7;
} CustomPollSensor;

/* 一条总线上挂载的全部传感器列表。 */
typedef struct {
  uint8_t bus_id;
  uint8_t count;
  PollSensor sensors[SENSOR_PER_BUS_MAX];
} PollList;

/* 板级拓扑常量：
 * g_poll_lists 定义“整板有哪些点位”；
 * g_custom_poll_list 定义“只采部分点位时要采哪些”。
 */
extern const SoftI2CBusPins g_bus_pins[SENSOR_BUS_COUNT];
extern const PollList g_poll_lists[SENSOR_BUS_COUNT];
extern const CustomPollSensor g_custom_poll_list[];
extern const uint8_t g_custom_poll_count;

#endif /* SENSOR_TOPOLOGY_H */
