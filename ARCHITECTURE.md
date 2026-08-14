# 固件架构说明

## 当前设计原则

当前固件定位为原始传感器采集节点。MCU 负责可靠地初始化器件、读取寄存器、
组帧、校验和 DMA 发送，不负责解释数据含义。这样可以先验证硬件、I2C 和通信链路，再依据
整机标定数据在上位机实现算法。

```text
main.c
  -> SensorApp
       -> Mlx90393 / Lsm6dsow -> BspI2c -> HAL I2C
       -> SensorProtocol -> ComCrc8Ccitt
       -> BspTime -> TIM2 1000 Hz fixed frame tick
       -> BspUartDma -> HAL UART/GPDMA

H503Flash -> HAL Flash
```

## 原始采集流程

默认打开 `BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND=1` 时，MLX90393 使用 burst mode 连续转换，
应用层按 `BOARD_TIM2_FIXED_SEND_DIVIDER` 对 TIM2 节拍分频后尝试 RM 读取最新结果；当前默认
分频为 2，即约 500 Hz 上报。若读回状态表示数据未就绪，则保留上一份有效样本，因此高频上报中
允许出现重复样本。将该宏设为 `0` 后，固件回到原来的流水式单次测量：周期 N 读取周期 N-1
已启动的结果，然后立即启动下一轮转换。

两颗 LSM6DSOW 在同一周期顺序执行 14 字节 burst read，得到温度、gyro XYZ 和 accel XYZ 原始值。
三颗传感器按固定顺序写入每一帧。

UART DMA 与采样解耦。DMA 使用独立发送缓冲区，OUT FIFO 只保存尚未开始发送的帧；启动 DMA
时先把最早待发帧复制到 DMA 缓冲，再立刻从 FIFO 移除。指腹串联方案中本机帧和 IN 口帧共用
同一个 OUT FIFO；同一来源已有待发帧时，新帧覆盖旧待发帧，不继续占用新槽位，不同来源仍按
入队先后发送。FIFO 只能吸收短时抖动，满且无法同源覆盖时淘汰最旧的未发送帧给新帧让位；
若连可淘汰帧也没有才丢新帧并计数，不等待队列腾空。IN 口解析每轮有字节预算限制，避免连续
输入长期占住主循环。

USART1 IN 口使用 GPDMA1 Channel 1 做 8192 字节大块接收，主循环按 DMA 剩余计数计算写入位置
并消费字节。这样高负载串联时不再每字节进入一次 HAL 接收中断；若主循环长期落后到 DMA 写
指针追上读指针，则丢弃最旧字节并保留最新数据，保证固件继续运行。

## TIM2 周期同步

TIM2 使用 240 MHz APB1 Timer 时钟，配置 `PSC=2399`。默认 1000 Hz 分支下 `ARR=99`，
每 1 ms 进入更新中断；`BspTime_OnTim2Elapsed()` 每次把 32 位毫秒计数增加 1，并记录一个
待主循环消费的发帧节拍。应用层默认 `BOARD_TIM2_FIXED_SEND_DIVIDER=2`，因此实际约 500 Hz
上报。旧 10 ms 分支下 `ARR=199`，保持 500 Hz、2 ms 时间基准。

固定发帧节拍不累计补发：主循环落后时只保留一个待处理事件，下一轮发送最新快照。MLX burst
运行期读取使用 2 ms 短超时，连续失败后进入 20 ms 重试冷却，避免单颗异常器件长时间阻塞
USART1 IN 解析和 OUT 队列发送。

默认分支下，应用层消费 TIM2 发帧节拍，并在每次进入采集周期时只读取一次 `now`。该值先写入
`SensorProtocolInput.tick_ms`，再顺序读取三颗 MLX 的 burst 结果，最后把本周期全部数据编码到
同一帧。因此时间戳表达的是整帧的采样批次时间，避免不同传感器分别取时造成上位机对齐困难。

## 40 字节上行帧

所有多字节整数按大端编码。协议中偏移 2 的字段叫传感器域，偏移 3 的字段叫传感器 id。
当前传感器域为 `1`，传感器 id 默认为 `0x00`。指尖磁铁方案传感器类型为 `0x01`；
指腹磁铁方案开启 USART1 IN 口并使用传感器类型 `0x02`：

| 偏移 | 长度 | 内容 |
| ---: | ---: | --- |
| 0 | 2 | 帧头 `0x1A 0x2B` |
| 2 | 1 | 传感器域，当前 `1` |
| 3 | 1 | 传感器 id，当前 `0x00` |
| 4 | 1 | 传感器类型，指尖 `0x01`，指腹 `0x02` |
| 5 | 1 | 当前 MLX90393 `GAIN_SEL`，范围 0～7，当前为 `0x05` |
| 6 | 2 | 总帧长 `40`，包含帧头至帧尾全部字节 |
| 8 | 4 | HAL 毫秒时间戳 `uint32_t` |
| 12 | 2 | 报文长度 `24`，只统计载荷 |
| 14 | 24 | 三颗 MLX，每颗 8 字节 |
| 38 | 1 | CRC8-CCITT，覆盖偏移 0 至 37 |
| 39 | 1 | 帧尾 `0x3C` |

单颗 MLX 的 8 字节顺序：

```text
Temperature:int16, X:int16, Y:int16, Z:int16
```

当前默认不写入 IMU 数据：指尖磁铁方案暂不上报 IMU，指腹磁铁方案硬件上没有 IMU。
该字节是三颗 MLX 共用的 `GAIN_SEL`，当前值为 `0x05`。传感器类型是前一字节，
两者不可混淆。
协议不包含浮点数、姿态角、双 IMU 差值或磁干扰判断结果。

CRC 使用 `CRC8-CCITT`：Poly=`0x07`、Init=`0x00`、输入/输出不反转、
XorOut=`0x00`；标准字符串 `123456789` 的结果为 `0xF4`。CRC 覆盖帧头第一个字节至
载荷最后一个字节，不包含 CRC 自身和帧尾。

## 算法模块状态

`Core/mid/Inc|Src/orientation.*` 和 `Core/mid/Inc|Src/magnetic_interference.*` 当前保留为前期草稿，但：

- 不被 `sensor_app.c` 包含。
- 不被 `sensor_protocol.h` 包含。
- 不加入当前 CMake `target_sources`。
- 不影响固件输出帧。

后续启用前需先定义传感器坐标系、安装方向、零偏、温漂、干扰阈值和上位机/固件职责边界。

## Flash 隔离

配置区为 `0x0801E000` 至 `0x0801FFFF`，对应 Bank 2 Sector 7。链接脚本只允许应用使用前
120 KB，从链接阶段避免代码覆盖配置区。

## CubeMX 与 CMake

当前 CubeMX 使用集中式生成：I2C、UART、GPIO、ICACHE 和 TIM 初始化函数及 HAL 句柄均位于
`main.c`。BSP 通过 `bsp_handles.h` 声明句柄，不依赖不存在的 `i2c.h/usart.h`。DMA 和 I2C
中断连接位于 USER CODE 区。重新生成后应复查根 CMake 自定义源码列表、这些 USER CODE 连接点
以及链接脚本 `STM32H503xx_FLASH.ld`。

