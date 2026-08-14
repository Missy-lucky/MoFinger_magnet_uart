# 灵巧手指尖原始传感器固件

本工程面向 STM32H503KBU6 灵巧手指磁铁传感器板。当前默认指尖磁铁方案只负责通过 I2C
采集三颗 MLX90393 的原始数据，再通过 LPUART1 TX DMA 输出。磁干扰判断、姿态识别、滤波、
标定和融合算法不在 MCU 内执行，后续可在上位机或新固件版本中实现。

## 当前硬件拓扑

- MCU：STM32H503KBU6，128 KB 双 Bank Flash，32 KB RAM。
- I2C1：PB6/SCL、PB5/SDA，400 kHz。
- MLX90393：U4 `0x0C` 为主磁传感器，U6 `0x0D`、U8 `0x0E` 为磁干扰参考传感器。
- LSM6DSOW：指尖硬件保留 `0x6A`、`0x6B`，当前默认不上报；指腹磁铁方案硬件上没有 IMU。
- LPUART1：PA3/TX、PA2/RX；当前 921600 baud，TX 使用 GPDMA1 Channel 0。
- USART1：PA1/RX、PA9/TX；只有 `BOARD_ENABLE_USART1_IN=1` 的指腹磁铁方案把它作为 IN 口。
- TIM2：240 MHz 输入时钟，`PSC=2399`；默认 `ARR=99`，以 1000 Hz 产生固定发帧节拍。
- Flash 配置区：`0x0801E000` 至 `0x0801FFFF`，即 Bank 2 Sector 7。

## 当前功能边界

### 方案和传感器输出宏

`Core/bsp/Inc/board_config.h` 使用 `BOARD_ENABLE_USART1_IN` 区分两套固件：默认 `0`
为指尖磁铁方案，不开 IN 口，传感器类型 `0x01`；定义为 `1` 时为指腹磁铁方案，
USART1 作为 IN 口，传感器类型 `0x02`，且该方案硬件上没有 IMU。

`Core/mid/Inc/sensor_protocol.h` 提供两个编译期宏：

| 模式 | `SENSOR_OUTPUT_ENABLE_MLX` | `SENSOR_OUTPUT_ENABLE_IMU` | 帧长 |
| --- | ---: | ---: | ---: |
| 只输出三颗 MLX（当前默认） | `1` | `0` | 40 字节 |
| 只输出两颗 IMU | `0` | `1` | 44 字节 |
| MLX 和 IMU 全部输出 | `1` | `1` | 68 字节 |

被关闭的传感器类型不会初始化、不会周期读取，也不会写入上行载荷。两个宏不能同时为 `0`。
指腹磁铁方案没有 IMU，不能打开 `SENSOR_OUTPUT_ENABLE_IMU`。
切换后需要重新编译固件，并让上位机按帧头中的总帧长和载荷长度解析对应格式。

固件当前执行：

1. 初始化三颗 MLX90393。
2. 默认用 TIM2 1000 Hz 固定节拍，按当前分频约 500 Hz 发帧，并从 MLX90393 burst mode 连续读取原始 T、X、Y、Z。
3. 指腹磁铁方案从 USART1 IN 口接收下一级合法帧，并和本机帧一起进入 LPUART1 OUT 队列，按先到先发。
4. 将当前 MLX90393 `GAIN_SEL` 写入协议对应字段。
5. 每个 TIM2 发帧节拍只读取一次时间戳，三颗 MLX 的本周期数据共用该时间戳。
6. 将同一周期原始数据编码成通信协议规定的 40 字节大端帧。
7. 计算 CRC8-CCITT。
8. 通过 LPUART1 TX DMA 异步输出。
9. 提供 H503 Flash 擦除、写入、读取和校验接口。

固件当前不执行：

- 主磁传感器与参考磁传感器之间的磁干扰判断。
- MLX 零偏、温漂、轴向或安装姿态标定。
- IMU roll、pitch、yaw 解算。
- 双 IMU 融合、一致性判断或运动状态识别。
- 低通、卡尔曼、互补滤波等复杂数据处理。

当前默认只输出三颗 MLX 的原始值，上述算法可由上位机根据实际标定数据实现。

## 代码分层

```text
Core/
  com/   公共层：Inc 公开头文件，Src CRC8 实现
  bsp/   BSP 层：Inc 板级接口，Src 时间/I2C/UART DMA/Flash 实现
  dri/   驱动层：Inc 器件接口，Src MLX90393/LSM6DSOW 实现
  mid/   中间层：Inc 协议接口，Src 原始上行协议编码
  app/   应用层：Inc 应用接口，Src 参数、周期采集和发送编排
  Inc/   CubeMX 生成头文件
  Src/   CubeMX 生成源文件和最小应用入口
```

当前有效依赖方向：

```text
App -> SensorProtocol -> ComCrc8Ccitt
App -> Mlx90393 / Lsm6dsow -> BspI2c
App -> BspTime(TIM2 1000 Hz 或旧 500 Hz) / BspUartDma
H503Flash -> STM32 HAL Flash
```

`orientation.*` 和 `magnetic_interference.*` 是前期算法草稿，当前不进入 CMake 构建，也不在
运行调用链中使用。后续恢复算法前应先确定标定方法、坐标系和产品阈值。

## MLX90393 采集配置

当前项目默认配置以 `#define` 集中在 `Core/app/Inc/sensor_config.h`：`HALLCONF=0xC`、
`GAIN_SEL=5`、`RES_X/Y/Z=1`、`DIG_FILT=2`、`OSR=0`、`OSR2=0`、`TCMP_EN=0`；
`sensor_config.c` 的 `g_sensor_mlx_config` 只引用这些宏。
`SENSOR_MLX_BURST_SEL=0x0F` 表示温度和 XYZ 全部转换，`SENSOR_MLX_BURST_DATA_RATE=0`
表示连续 burst。`BOARD_TIM2_FIXED_SEND_DIVIDER` 控制 1000 Hz 固定节拍分支的 UART 上报分频，
当前默认 `2`，即约 500 Hz 上报；改为 `1` 可回到 1000 Hz 单板调试。TXYZ 典型转换时间约
1.84 ms，最大输出速率约 493 Hz；因此 1000 Hz 发帧时会按预期出现重复样本。

驱动通过 `Mlx90393Config` 接收配置，并保存到每颗器件上下文；如需改增益、分辨率、滤波或
过采样，只修改 `sensor_config.h` 中对应的 `#define`，不需要进入 `sensor_config.c`、驱动源文件
或任何子对象。DRI 层保留 `HALLCONF=0xC` 和 `0x0` 两套
完整灵敏度表，`Mlx90393_GetSensitivity()` 可按 GAIN/RES 查询三轴 `uT/LSB`。当前固件仍只
上传原始值，不调用该接口执行浮点换算。

驱动按数据手册保留 Reg0 高 8 位 `ANA_RESERVED_LOW/BIST`，在低 8 位设置
`GAIN_SEL/HALLCONF`，根据配置结构体生成 Reg1/Reg2，并对三个寄存器执行写后回读校验。

## LSM6DSOW 采集配置

- 加速度计：104 Hz、正负 16 g。
- 陀螺仪：104 Hz、正负 2000 dps。
- BDU 开启，保证同一轴高低字节一致。
- IF_INC 开启，从 `OUT_TEMP_L` 开始连续读取 14 字节。

以上参数同样以 `#define` 集中在 `Core/app/Inc/sensor_config.h`（`SENSOR_IMU_XL_ODR/XL_FS`、
`SENSOR_IMU_G_ODR/G_FS`、`SENSOR_IMU_CTRL3_C`），组合成 `g_sensor_imu_config` 的三个 CTRL
寄存器字节；`Lsm6dsow_Init()` 按传入配置写入，改量程或输出速率只编辑对应的 `#define`。

协议输出保持寄存器原始 `int16_t`，不换算为 g、dps 或欧拉角。

## UART DMA

40 字节帧以 500 Hz、8N1 发送时线路负载约为 200 kbit/s，低于当前 921600 baud。DMA 忙时
只跳过或覆盖尚未发送的旧帧，MLX burst 连续转换继续运行。上位机串口参数必须与固件一致。

指腹串联方案中，USART1 IN 口使用 GPDMA1 Channel 1 进行 8192 字节大块 DMA 接收，主循环从
DMA 缓冲中按读写位置消费字节，不再每字节进入一次 HAL 接收中断；后续帧长或串联数量增加时
可调整 `BOARD_UART_IN_DMA_BUFFER_SIZE`。本机帧和 IN 口帧仍进入同一个 OUT 队列，按先到先发；
1000 Hz 分支把队列加深到 32 帧以吸收短时抖动。DMA 使用独立发送缓冲区，OUT 队列只保存尚未开始发送的帧；
启动 DMA 时把最早待发帧复制到 DMA 缓冲后立即从队列移除。同一来源
`(传感器域, 传感器id, 传感器类型)` 若已有待发帧，新帧会覆盖旧待发帧，不继续占用新队列槽；
不同来源仍按入队先后发送。队列满且没有同源槽可覆盖时，会淘汰最旧的未发送帧给新帧让位；
若连可淘汰帧也没有才丢新入队帧并计数。IN 口解析每轮最多处理
`BOARD_UART_IN_PROCESS_BYTE_BUDGET` 字节，避免连续输入长期占住主循环。注意 40 字节帧在
8N1 下约 `400 bit/frame`：500 Hz 一路约 200 kbit/s，四路约 800 kbit/s；1000 Hz 三路会
超过 921600 baud 物理带宽。长期超带宽运行时只能靠同源覆盖/淘汰/丢帧保持系统不阻塞，
不能保证每帧都传出。

## TIM2 同步时间基准

TIM2 输入时钟为 240 MHz，预分频后为 `240 MHz / (2399 + 1) = 100 kHz`。默认打开
`BOARD_ENABLE_TIM2_1000HZ_FIXED_SEND=1` 时，`ARR + 1 = 100`，更新频率为 `1000 Hz`、周期
为 `1 ms`；BSP 在中断中把 32 位毫秒计数增加 1，并记录一个待主循环消费的固定发帧节拍。
应用层再按 `BOARD_TIM2_FIXED_SEND_DIVIDER` 分频后实际发帧，当前默认约 500 Hz。
将该宏设为 `0` 后，固件回到 `ARR=199`、500 Hz 时间基准和原来的 10 ms deadline 分支。

每个固定发帧节拍在开始采集前只捕获一次时间戳。三颗 MLX 虽然必须在 I2C 总线上顺序读取，
但它们作为同一批次写入一帧并共用该时间戳，上位机应把这一帧解释为同一采样周期。
这属于周期级同步，不代表三颗器件在同一微秒完成物理采样。

1000 Hz 分支不会补发积压的 TIM2 tick：如果主循环因为 I2C 或 DMA 忙错过多个 tick，只保留
一个待处理发帧事件，下一次直接发送最新快照。MLX 运行期 RM 使用 2 ms 短超时，连续失败后
进入 20 ms 重试冷却，期间复用上一份有效样本，避免异常器件长时间卡住串联转发。

## Flash 使用规则

- 链接脚本将应用 Flash 限制为前 120 KB。
- 最后 8 KB 为 Bank 2 Sector 7 配置区。
- 擦除单位为 8 KB，编程单位为 16 字节 Quad-word。
- 写入 offset 必须 16 字节对齐。
- 写前检查目标为擦除态，写后自动回读比较。

## 构建说明

VS Code/CMake 必须选择 `Debug` 或 `Release` preset，并使用 `arm-none-eabi-gcc`，不能使用
Visual Studio/MSVC。根 `CMakeLists.txt` 已显式加入当前原始采集模块；CubeMX 外设初始化集中在 `main.c`。

2026-07-29 已使用 STM32Cube 工具包中的 GNU Arm 14.3.1 完成 Debug 构建，生成 `build/Debug/Mosense_AY01JA.elf`。
当前协议使用单字节 CRC8-CCITT；Flash 链接区域为 120 KB。仍需在目标板验证 I2C、DMA 和 Flash。

## 进一步阅读

## 原始数据采集上位机

可视化界面直接运行：

```powershell
python mlx90393_collector_gui.py
```

界面支持自动选择 USB 串口、连接/断开、三颗 MLX 的 T/X/Y/Z 实时显示、选定传感器 XYZ 曲线、采样率统计以及 CSV 开始/停止保存。

工程根目录的 `mlx90393_collector.py` 用于烧录后验证和持续采集。它会校验帧头、传感器域、传感器 id、传感器类型 `0x01/0x02`、帧长、载荷长度和 CRC8，并将主机时间、固件毫秒时间戳、传感器类型、GAIN_SEL 以及三颗 MLX 的温度和 X/Y/Z 写入 UTF-8-SIG CSV。上位机按 `25 + T/45.2` 将帧内温度差值换算为摄氏度；XYZ 保持当前配置下的有符号原始值。

```powershell
python -m pip install pyserial
python mlx90393_collector.py --port COM8 --output capture.csv --seconds 30
```

烧录验证时先只开一个串口工具或采集器，避免多个进程同时占用 COM 口；当前默认预期约 500 帧/秒，`tick_ms` 按约 2 ms 递增。

- [ARCHITECTURE.md](ARCHITECTURE.md)：原始数据流和当前 40 字节协议布局。
- [模块与函数说明.md](模块与函数说明.md)：逐文件、逐函数说明。
- [数据手册核对记录.md](数据手册核对记录.md)：器件手册页码和配置推导。
- [Bootloader与Flash维护说明.md](Bootloader与Flash维护说明.md)：参数地址判定、整 Sector 保留更新和调用示例。
