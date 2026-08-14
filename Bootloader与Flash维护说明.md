# Bootloader 参数区与 Flash 维护说明

## 地址先按数值解释

STM32H503 内部 Flash 范围为 `0x08000000` 至 `0x0801FFFF`，每个物理页/Sector 为
`0x2000`（8 KB）。地址与页号的关系为：

```text
全局页号 = (地址 - 0x08000000) / 0x2000
页内偏移 = (地址 - 0x08000000) % 0x2000
```

三个容易混淆的地址含义完全不同：

| 地址 | 所在位置 |
| --- | --- |
| `0x08001800` | 全局页 0 内偏移 0x1800，确实位于前 8 KB Bootloader 区内 |
| `0x08018000` | 全局页 12 起点，即 Bank 2 Sector 4，不在 Bootloader/A 区 |
| `0x0801E000` | 全局页 15 起点，即 Bank 2 Sector 7 |
| `0x080018000` | 数值等于 `0x80018000`，超出 H503 内部 Flash 范围 |

架构图中 Bootloader 从 `0x08000000` 开始，占全局页 0 至 2；A code 从 `0x08006000` 开始。
因此，若参数确实位于 Bootloader 内，最可能想表达的是 `0x08001800`。若地址确实是
`0x08018000`，它就是全局页 12，并且按图中 B code 从 `0x0800C000` 延伸到页 14 的规划，会与
B code 区域重叠。最终地址需要与 Bootloader/A/B 三份链接脚本一起确认。

## 当前代码采取的安全策略

在最终地址确认前，`H503Flash_UpdatePreserve()` 只允许操作链接脚本已经明确保留的全局页 15：

```text
0x0801E000 - 0x0801FFFF
Bank 2 Sector 7
```

这样不会误擦 Bootloader、A code 或 B code。最终改地址时必须同步修改：

1. `board_config.h` 的参数区起始地址。
2. `h503_flash.c` 的 Bank/Sector。
3. Bootloader、A code、B code 的链接脚本。
4. 参数结构在 Sector 内的 offset 定义。

## 新接口用途

```c
AppStatus H503Flash_UpdatePreserve(
    uint32_t offset,
    const void *data,
    uint32_t length,
    uint8_t *sector_backup,
    uint32_t backup_size);
```

内部顺序为：

```text
完整读取 8 KB到RAM
-> 在RAM中修改目标字段
-> 内容未变化则直接返回
-> 擦除完整8 KB Sector
-> 写回修改后的完整8 KB
-> 比较完整8 KB
```

调用者应静态分配备份区，不能使用 8 KB 局部栈数组：

```c
static uint8_t g_parameter_sector_backup[H503_FLASH_SECTOR_BACKUP_SIZE];

AppStatus BootFlags_ClearRunFlag(uint32_t flag_offset)
{
    const uint8_t cleared = 0x00u;
    return H503Flash_UpdatePreserve(flag_offset,
                                    &cleared,
                                    sizeof(cleared),
                                    g_parameter_sector_backup,
                                    sizeof(g_parameter_sector_backup));
}
```

`offset` 是相对于参数 Sector 起点的偏移，不是绝对 Flash 地址。例如参数区最终为
`0x0801E000`，标志绝对地址为 `0x0801E006`，调用时 offset 为 `0x0006`。

## 能解决和不能解决的问题

该接口能避免“为了清一个标志，误删同一 Sector 中的校准参数”。它不能保证掉电安全：Sector
擦除后、完整写回前如果掉电，RAM 备份也会丢失。需要掉电安全时应规划两个参数 Sector，采用
主备镜像、序号、CRC 和最后提交标志。
