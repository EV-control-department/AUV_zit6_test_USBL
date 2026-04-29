# 惯导初始化说明

本文说明当前固件中 NAV-300 惯导的初始化流程、串口角色、协议格式和代码入口。内容以当前 CMake 工程里的实现为准。

## 1. 串口角色

- UART7：惯导数据接收口，使用 DMA 循环接收。
- UART3：惯导初始化命令发送口，用于下发模式切换指令。
- USART2：micro-ROS 通信口，与惯导初始化无关，但会影响系统整体调度和日志发布。

当前配置为：

- UART7 波特率：256000
- UART3 波特率：256000
- USART2 波特率：921600

## 2. 初始化顺序

惯导初始化在 `UserApp_ControlTask()` 中执行，入口是 `INS_Driver::init()`。

### 2.1 开启 UART7 接收

先调用 `rx_port_.startReceive()`，通过 DMA 启动 UART7 的循环接收。

这一步是惯导数据能否进入缓冲区的前提。如果 UART7 没有开始接收，后续解析函数不会拿到任何字节。

### 2.2 下发惯导模式切换命令

随后通过 UART3 发送 14 字节初始化命令：

```cpp
uint8_t init_cmd[14] = {0xFC, 0xCF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xFD, 0xDF};
HAL_UART_Transmit(tx_uart_, init_cmd, 14, 20);
```

这条命令的作用是让惯导进入当前工程需要的导航模式。发送口必须是 UART3，不能误发到 UART7。

### 2.3 进入循环解析

初始化完成后，`UserApp_ControlTask()` 以 50Hz 调用 `ins_driver.update(current_nav_state)`，持续从 UART7 的 DMA 缓冲中读取数据并解析。

## 3. 惯导数据帧格式

当前解析逻辑按 NAV-300 的外部组合导航帧处理：

- 帧头：`0xFA 0xAF`
- 校验：对前 130 字节做 XOR，校验值放在 `packet_buf_[130]`
- 帧尾：`0xFB 0xBF`
- 总长度：132 字节

### 3.1 关键字段

- `2~13`：roll / pitch / yaw
- `54~61`：X / Y 位置
- `62~65`：Z 深度
- `115`：DVL 状态位，bit7 为有效标志
- `129`：导航模式

## 4. 代码入口

初始化相关代码主要在以下位置：

- [UserApp/Peripherals/INS_Driver.hpp](../UserApp/Peripherals/INS_Driver.hpp)
- [UserApp/Application/AppMain.cpp](../UserApp/Application/AppMain.cpp)
- [Core/Src/usart.c](../Core/Src/usart.c)
- [Core/Src/stm32h7xx_it.c](../Core/Src/stm32h7xx_it.c)

## 5. 启动时序建议

建议的上电时序如下：

1. 先保证 UART7、UART3 硬件和线缆连接正确。
2. 启动固件后，`UserApp_ControlTask()` 立即执行 `INS_Driver::init()`。
3. UART7 开始 DMA 接收。
4. UART3 下发初始化命令。
5. 若惯导正常返回数据，`update()` 会在 50Hz 主循环中持续解包。

## 6. 常见问题

### 6.1 没有惯导数据

优先检查以下内容：

- UART7 是否真的接到了惯导 RX。
- UART3 是否真的接到了惯导 TX。
- UART7 波特率是否和惯导一致。
- DMA1_Stream2 是否正常工作。
- 初始化命令是否在上电后被正确发送。

### 6.2 只看到串口有数据，但解析不到帧

优先检查：

- 数据帧是否是 `FA AF ... FB BF` 这种格式。
- 校验是否按前 130 字节 XOR。
- 当前是否处于外部组合导航模式。

### 6.3 方向或坐标映射不对

当前代码映射为：

- `state.y = X`
- `state.x = -Y`
- `state.z = Z`
- `state.yaw = yaw`

如果实际坐标系定义不同，需要在 `decodePacket()` 里调整。

## 7. 调试建议

- 如果没有数据，先确认 UART7 DMA 缓冲是否在变化。
- 如果有字节但没有完整帧，优先检查协议头尾和校验。
- 如果惯导能回数据但姿态不对，优先检查协议偏移和单位。

## 8. 与 micro-ROS 的关系

惯导初始化不依赖 micro-ROS，但惯导数据会被后续的控制与发布任务使用。

- 位置状态会发布到 `/zit6/state/pose`
- 速度状态会发布到 `/zit6/state/velocity`
- INS 状态会发布到 `/zit6/state/ins_info`
- 心跳会发布到 `/zit6/heartbeat`

这几个话题的刷新节奏由主循环分频控制。