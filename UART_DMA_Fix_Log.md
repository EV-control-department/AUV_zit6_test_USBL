# STM32 Micro-ROS UART DMA 修复记录

## 1. 问题背景
系统在运行 micro-ROS 时容易进入 `HardFault_Handler`，并且经常出现**“一直不能和上位机通信”**的现象。

经过排查，发现根本原因为：**USART2 全局中断未开启**。
在使用 DMA 接收数据时，如果上位机发送过快或系统负载较高，容易触发 UART 的硬件溢出错误（Overrun Error, ORE）。由于未开启全局中断，HAL 库的错误处理函数无法介入清除 ORE 标志位，导致 DMA 永久停止接收数据，造成数据流断裂或错位。数据错位后，micro-ROS 解析器可能会分配极大的异常内存，从而导致 `std::bad_alloc` 内存溢出或直接引发 `HardFault`。

## 2. 修改内容

为了解决该问题，我们对代码进行了以下 3 处关键修改：

### 2.1 开启 USART2 全局中断
**修改文件**：`Core/Src/usart.c`
**修改位置**：`HAL_UART_MspInit` 函数中 `USART2` 分支。
**说明**：添加了中断使能代码，并严格设置优先级为 5（等于 FreeRTOS 的 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，防止调用 FreeRTOS API 时崩溃）。
```c
    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
```

### 2.2 暴露 DMA 接收 Buffer 变量
**修改文件**：`micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c`
**修改位置**：`dma_buffer` 声明处。
**说明**：去除了 `static` 关键字，使得其在其它文件（即 `usart.c`）的错误回调函数中可以被访问，以便用于 DMA 重启。
```diff
- static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer")));
+ uint8_t dma_buffer[UART_DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer")));
```

### 2.3 添加 UART 错误回调函数（实现自动恢复）
**修改文件**：`Core/Src/usart.c`
**修改位置**：文件末尾的 `USER CODE BEGIN 1` 区域。
**说明**：当 HAL 库捕获到 ORE 溢出或帧错误时，会自动清理标志位并**中止** DMA。因此我们通过实现弱函数 `HAL_UART_ErrorCallback`，在检测到错误后重新开启 `USART2` 的 DMA 接收，实现通信的自我修复（Self-Healing），防止假死。
```c
/* USER CODE BEGIN 1 */
extern uint8_t dma_buffer[];

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    /* 发生溢出(ORE)等错误时，HAL库会中止DMA，我们需要将其重新开启以防通讯假死 */
    HAL_UART_Receive_DMA(huart, dma_buffer, 2048);
  }
}
/* USER CODE END 1 */
```

## 3. 后续验证建议
1. 请重新编译并烧录固件到 STM32H743。
2. 启动 `micro_ros_agent` 进行长时间高频度的数据交互测试。
3. 观察是否还会出现卡死无法通信，或者掉入 `HardFault_Handler` 的问题。目前通过上述机制，一旦发生错乱也会立刻清空错误并重启恢复。
