#include "sensor_topology.h"

/* 板级拓扑常量表：
 * 这里定义每条软 I2C 总线的引脚绑定，以及每条总线上实际挂载的 MLX90393 名称/地址。
 * 若硬件改版，优先修改这个文件，而不是散改采样逻辑。
 */

const SoftI2CBusPins g_bus_pins[SENSOR_BUS_COUNT] = {
    /* bus0 ~ bus11 与硬件上 12 组 SCL/SDA 引脚一一对应。 */
    {SCL0_GPIO_Port, SCL0_Pin, SDA0_GPIO_Port, SDA0_Pin},
    {SCL1_GPIO_Port, SCL1_Pin, SDA1_GPIO_Port, SDA1_Pin},
    {SCL2_GPIO_Port, SCL2_Pin, SDA2_GPIO_Port, SDA2_Pin},
    {SCL3_GPIO_Port, SCL3_Pin, SDA3_GPIO_Port, SDA3_Pin},
    {SCL4_GPIO_Port, SCL4_Pin, SDA4_GPIO_Port, SDA4_Pin},
    {SCL5_GPIO_Port, SCL5_Pin, SDA5_GPIO_Port, SDA5_Pin},
    {SCL6_GPIO_Port, SCL6_Pin, SDA6_GPIO_Port, SDA6_Pin},
    {SCL7_GPIO_Port, SCL7_Pin, SDA7_GPIO_Port, SDA7_Pin},
    {SCL8_GPIO_Port, SCL8_Pin, SDA8_GPIO_Port, SDA8_Pin},
    {SCL9_GPIO_Port, SCL9_Pin, SDA9_GPIO_Port, SDA9_Pin},
    {SCL10_GPIO_Port, SCL10_Pin, SDA10_GPIO_Port, SDA10_Pin},
    {SCL11_GPIO_Port, SCL11_Pin, SDA11_GPIO_Port, SDA11_Pin},
};

/* 按 bus 列出的完整轮询拓扑。 */
const PollList g_poll_lists[SENSOR_BUS_COUNT] = {
    /* 每个 PollList 的 sensors[] 顺序就是该 bus 上 slot 的逻辑顺序。 */
    {0u, 4u, {{"U50", 0x0Cu}, {"U10", 0x0Du}, {"U1", 0x0Eu}, {"U11", 0x0Fu}}},
    {1u, 4u, {{"U13", 0x0Cu}, {"U9", 0x0Du}, {"U2", 0x0Eu}, {"U12", 0x0Fu}}},
    {2u, 3u, {{"U6", 0x0Cu}, {"U4", 0x0Du}, {"U3", 0x0Eu}, {"", 0u}}},
    {3u, 4u, {{"U15", 0x0Cu}, {"U8", 0x0Du}, {"U7", 0x0Eu}, {"U14", 0x0Fu}}},
    {4u, 3u, {{"U19", 0x0Cu}, {"U16", 0x0Eu}, {"U18", 0x0Fu}, {"", 0u}}},
    {5u, 4u, {{"U23", 0x0Cu}, {"U21", 0x0Du}, {"U20", 0x0Eu}, {"U22", 0x0Fu}}},
    {6u, 4u, {{"U27", 0x0Cu}, {"U25", 0x0Du}, {"U24", 0x0Eu}, {"U26", 0x0Fu}}},
    {7u, 4u, {{"U31", 0x0Cu}, {"U29", 0x0Du}, {"U28", 0x0Eu}, {"U30", 0x0Fu}}},
    {8u, 4u, {{"U17", 0x0Cu}, {"U33", 0x0Du}, {"U32", 0x0Eu}, {"U34", 0x0Fu}}},
    {9u, 4u, {{"U39", 0x0Cu}, {"U37", 0x0Du}, {"U36", 0x0Eu}, {"U38", 0x0Fu}}},
    {10u, 3u, {{"U40", 0x0Eu}, {"U41", 0x0Du}, {"U42", 0x0Fu}, {"", 0u}}},
    {11u, 4u, {{"U47", 0x0Cu}, {"U45", 0x0Du}, {"U44", 0x0Eu}, {"U46", 0x0Fu}}},
};

/* 自定义轮询清单：
 * 开启 APP_CUSTOM_SENSOR_LIST_EN 时，collector 只会处理这些点位。
 */
const CustomPollSensor g_custom_poll_list[] = {
#if (APP_MASK_U17_EN == 0U)
    /* 自定义清单通常拿来做小规模验证或单点排障。 */
    {"U6", 2u, 0u, 0x0Cu},
    {"U32", 8u, 2u, 0x0Eu},
#endif
};

const uint8_t g_custom_poll_count =
    (uint8_t)(sizeof(g_custom_poll_list) / sizeof(g_custom_poll_list[0]));
