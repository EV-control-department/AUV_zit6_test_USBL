# 固件代码构建指南

本项目基于 CMake 进行构建，并集成了 Docker 来自动编译 micro-ROS 的静态库 (`libmicroros.a`)。

## 1. 编译依赖项
* **CMake** (建议版本 3.22+)
* **Ninja** 或 Make
* **GCC ARM Toolchain** (`arm-none-eabi-gcc`)
* **Docker** (用于在容器内构建 micro-ROS 依赖)

## 2. 内存分区与链接脚本
由于 STM32H743ZIT6 的特殊内存架构，为了防止 FreeRTOS 的堆与 micro-ROS 内存池导致 `DTCMRAM` (128KB) 溢出报错 (`std::bad_alloc`)，本项目的链接脚本 (`STM32H743XX_FLASH.ld`) 已被特殊修改：
* `.data`, `.bss`, 和 `._user_heap_stack` 均被重映射到了 `RAM` (即 AXI SRAM，起始地址 `0x24000000`，容量 512KB)。

## 3. 构建步骤

### 初次构建 / 修改了 micro-ROS 配置 (colcon.meta) 时
当你刚拉取代码，或者修改了需要微 ROS 支持的额外类型时，必须先使用 Docker 重建静态库：
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target microros_build
```

### 日常代码编译 (修改了应用层代码 `main.c`, `freertos.c` 等)
只要微 ROS 的静态库不需要更新，你只需执行标准的 CMake 编译即可：
```bash
cmake --build build
```
编译成功后，生成的 ELF 固件位于 `build/AUV_zit6.elf`。

## 4. 特别说明
* 我们修改了 `micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/library_generation.sh` 脚本，跳过了基于 Makefile 的 CFLAGS 提取，直接硬编码注入了 STM32H7 所需的 Cortex-M7 硬件浮点指令 (`-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard`)。如果未来更换芯片，请注意修改该脚本的 `RET_CFLAGS` 参数。
