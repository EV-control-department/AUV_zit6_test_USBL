# VS Code 软件调试与烧录指南

本项目已经为您配置好了基于 VS Code + Cortex-Debug + OpenOCD 的一键烧录与调试环境。

## 1. 必需的插件与软件
* **VS Code 扩展**: `Cortex-Debug` (作者: marus25)
* **烧录软件**: `openocd` (请确保可以在终端直接运行 `openocd --version`)
* **GDB 调试器**: `arm-none-eabi-gdb` (包含在 ARM GCC 工具链中)

## 2. 一键在线调试 (F5)
我们已在 `.vscode/launch.json` 中配置了 `Cortex Debug (OpenOCD)` 启动项。
1. 将 ST-Link 烧录器连接至电脑，并将调试线缆(SWD)接上 STM32H7 对应的引脚。
2. 在 VS Code 中按下 **F5** (或点击左侧调试面板的绿色启动按钮)。
3. 系统将自动执行以下流程：
   - 触发 `preLaunchTask` 自动执行 `cmake --build build` 以确保固件是最新版。
   - 启动 OpenOCD 服务端并连接开发板。
   - 自动擦除并烧录 `build/AUV_zit6.elf`。
   - 在 `main()` 函数的第一行打下断点并暂停。
4. 现在你可以使用 F10(单步)、F11(进入函数) 实时查看寄存器和变量了。

## 3. 仅烧录固件 (不开启调试模式)
如果你只想快速将最新代码烧录到板子上跑起来，我们配置了独立的 VS Code 任务。
1. 使用快捷键 **Ctrl + Shift + B**。
2. 在弹出的菜单中选择 **Flash Firmware (OpenOCD)**。
3. 系统会调用 `openocd` 命令一键刷入固件并自动复位运行。

## 4. 微 ROS 上位机通讯测试
烧录运行后，通过 TTL 转 USB (CH340N) 模块连接串口2 (波特率已被严格锁定为 `256000` bps，以防 ADUM1201ARZ 隔离器丢包)。
在上位机终端运行微 ROS 代理：
```bash
docker run -it --rm -v /dev:/dev --privileged microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 256000
```
然后新开一个终端，监听心跳数据：
```bash
ros2 topic echo /auv/heartbeat
```
