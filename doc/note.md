# Micro-ROS STM32H743ZIT6 部署技术笔记

## 1. 构建系统建议 (Build System)
*   **首选推荐**: **CMake**
*   **原因分析**: `micro_ros_stm32cubemx_utils` 官方高度依赖基于 GCC 工具链的 Docker 环境进行静态库 (`libmicroros.a`) 编译。MDK-ARM 的 ARMCC/ARMCLANG 编译器及其专有 C 库极易引发底层 POSIX 冲突、内存分配不兼容 (`std::bad_alloc`) 以及 Linker 冲突。CMake (结合 STM32CubeMX 生成器) 是最优雅且最稳定的方案。

## 2. 通讯链路设计 (Communication Link)
*   **物理链路**: STM32 (UART2 PA2/PA3) -> ADUM1201ARZ (数字隔离器) -> CH340N (TTL转USB) -> USB_HUB -> 笔记本电脑。
*   **硬件瓶颈限制**:
    *   根据佳立创原理图，电路上使用了 **ADUM1201ARZ-RL7** 数字隔离芯片。该芯片的 "A" 级版本最大数据传输速率被严格限制在 **1 Mbps**。
    *   后端的 **CH340N** 稳定极限通常在 2 Mbps 左右。
*   **关键结论**: USART2 的波特率**绝对不能超过 1 Mbps**，否则隔离器将产生严重的数据丢包或乱码（微 ROS 对数据流完整性极度敏感，一丁点乱码就会导致反序列化失败甚至程序崩溃）。建议将波特率设为 **256000 bps** 或 **460800 bps**。

## 3. 硬件外设配置检查 (Hardware Configuration - `AUV_zit6.ioc`)
*   **FreeRTOS**: CMSIS_V2，`configTOTAL_HEAP_SIZE` 60000 bytes，`micro_ros_task` 栈 3000 words (12KB)。(配置合理)
*   **USART2**: 异步模式。(需在生成代码前在 CubeMX 中设置上述建议的波特率)
*   **DMA**: `USART2_RX` (Circular, DMA1_Stream0), `USART2_TX` (Normal, DMA1_Stream1)。(完美匹配官方 `dma_transport.c` 的要求)

## 4. 内存爆炸/溢出问题分析 (Memory Overflow Analysis)
*   **现象溯源**: 用户曾遇到“内存爆炸” (Memory Overflow / `std::bad_alloc`) 问题。
*   **根本原因**: STM32H743 拥有复杂的内存分区。CubeMX 默认生成的工程，会将程序的 `.data` (全局变量) 和 `.bss` (未初始化的静态区，其中就包含了 FreeRTOS 庞大的 60KB Heap 数组) 统统塞进默认的首选内存段 **DTCMRAM (大小仅 128KB)**。当加载了微 ROS 的庞大依赖栈和 FreeRTOS 堆后，这 128KB 瞬间就会被撑爆，导致 Linker 报错或者运行时内存越界崩溃。
*   **核心解决方案**: 必须在生成代码后，手动修改 CMake 使用的链接器脚本 (`STM32H743ZITX_FLASH.ld`)，将主 RAM 区域从 `DTCMRAM` 强行重新映射到 `RAM_D1`（即 AXI SRAM，起始地址 `0x24000000`，拥有 **512KB** 的庞大连续空间）。

## 5. 调试手段 (Debugging)
*   **工具链**: 使用 OpenOCD + ST-Link，利用 SWD 接口进行调试。这对于没有多余串口打印 log 的情况来说是救命稻草。
