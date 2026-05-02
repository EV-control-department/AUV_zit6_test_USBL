Ran command: `openocd -f interface/stlink.cfg -c 'transport select hla_swd' -c 'adapter speed 1000' -f target/stm32h7x.cfg -c 'program build/Debug/AUV_zit6.elf verify reset exit'`
Ran command: `cmake --build build/Debug`
Viewed main.c:1-43

在上位机（AGX）端，为了让你的 RQT、命令行（`ros2 topic echo`）以及你自己写的 Python/C++ 节点能够识别这些自定义消息，你需要按照以下步骤进行编译和环境变量挂载：

### 1. 编译消息包
首先，你需要在 AGX 的 ROS 2 工作空间中编译这个消息定义包。假设你的工作空间在 `~/ros2_ws`（如果不是，请切换到对应的目录）：

```bash
# 1. 进入工作空间
cd ~/dev/2026-auv-sub/AUV_zit6_cmake

# 2. 编译自定义消息包
colcon build --packages-select zit6_interfaces
```

### 2. 需要 Source 的路径
编译完成后，你需要按顺序执行以下 `source` 命令。**建议将这些命令添加到你的 `~/.bashrc` 中**，这样每个新终端都会自动加载：

```bash
# 1. Source ROS 2 基础环境 (假设你用的是 Humble)
source /opt/ros/humble/setup.bash

# 2. Source 刚才编译出来的自定义消息环境
# 注意：路径要指向你刚才 colcon build 生成的 install 目录
source ~/dev/2026-auv-sub/AUV_zit6_cmake/install/setup.bash

# 3. (可选) 如果你安装了 micro-ROS Agent (如果是 Snap 安装则不需要这一步)
# source ~/microros_ws/install/setup.bash
```

---

### 3. 如何验证是否生效？
Source 完之后，你可以运行以下命令检查上位机是否已经“认识”了这些新消息：

```bash
# 检查消息类型是否存在
ros2 interface show zit6_interfaces/msg/ZitSetpoint
ros2 interface show zit6_interfaces/msg/ZitStatus
```

**如果你能看到 `control_key`、`seq`、`battery_voltage` 等字段输出，说明上位机环境已经完全就绪！**

### 4. 启动调试
现在你可以启动 Agent 并观察数据了：
1.  **启动 Agent**:
    ```bash
    micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600
    ```
2.  **查看状态反馈**:
    ```bash
    ros2 topic echo /zit6/state/status
    ```
3.  **使用 RQT 发送指令**:
    打开 `rqt` -> `Message Publisher` -> 选择 `/zit6/cmd/setpoint`。你会发现现在可以非常直观地输入各个维度的目标值了。

---

**💡 小提醒**：如果你之后又修改了 `.msg` 文件（比如增加了新字段），记得**两边都要更新**：
*   **固件侧**：运行 `cmake --build build --target microros_build` 并重新烧录。
*   **上位机侧**：重新运行 `colcon build --packages-select zit6_interfaces` 并重新 `source`。