# NAV-300 状态位映射表 (Status Mapping)

本文档定义了 NAV-300 组合导航系统在代码中解析并上发的状态位映射关系。

## 1. 导航模式 (Navigation Mode / imu_state)
对应协议中偏移地址 `129` (室外模式) 或 `66` (室内模式) 的字节。

| 数值 (Hex) | 状态名称 | 含义说明 | 备注 |
| :--- | :--- | :--- | :--- |
| **0x00** | 待机 (Standby) | 系统上电初始化中 | |
| **0x01** | 粗对准 (Coarse Alignment) | 正在进行初步找北 | 建议静止，耗时约 5-10min |
| **0x02** | 精对准 (Fine Alignment) | 正在进行高精度找北 | 建议静止，总耗时约 10-20min |
| **0x03** | SINS/GPS/DVL | 组合导航 (室外) | 利用 GPS/DVL/惯导进行全数据融合 |
| **0x04** | SINS/DVL | 组合导航 (室内/水下) | 利用 DVL/惯导进行融合，**常用模式** |
| **0x05** | MRU (Vertical Gyro) | 姿态模式 | 无外部速度参考，仅输出姿态/角速度 |
| **0xFF** | 系统故障 (Error) | 硬件或算法致命错误 | 需检查接线或重启 |

## 2. DVL 状态位 (DVL Status / dvl_state)
代码中通过 `packet_buf[115] >> 7` 提取（对应协议中传感器状态位的有效性判断）。

| 数值 | 状态 | 含义 |
| :--- | :--- | :--- |
| **1** | 有效 (Valid) | DVL 已上电并在水下正常锁底，速度数据可用 |
| **0** | 无效 (Invalid) | DVL 未开启、未入水或无法锁定池底/海床 |

---

## 3. micro-ROS 话题映射 (Topic Mapping)

系统中通过以下两个话题对外发布状态：

### A. `/zit6/state/ins_info` (std_msgs/UInt32)
该话题用于监控导航内部详细状态。
*   **Bit 15 - 8**: `imu_state` (见上表1)
*   **Bit 7 - 0**: `dvl_state` (见上表2)

**解析示例 (Python):**
```python
imu_mode = (msg.data >> 8) & 0xFF
dvl_valid = msg.data & 0xFF
```

### B. `/zit6/heartbeat` (std_msgs/UInt32)
该话题用于系统保活和快速模式监控。
*   **Bit 31 - 24**: `imu_state` (当前导航模式)
*   **Bit 23 - 0**: 系统运行时间戳 (单位: ms, 24位循环)

---

## 4. 控制话题 (Control Topic)

系统订阅以下话题用于接收控制指令：

### `/zit6/cmd/ins_command` (std_msgs/UInt8)

| 数值 (Dec) | 指令名称 | 行为说明 |
| :--- | :--- | :--- |
| **1** | DVL 开启 | 发送 DVL 电源开启指令 (**仅限水下执行**) |
| **2** | DVL 关闭 | 发送 DVL 电源关闭指令 |
| **3** | 惯导重启 | 强制惯导系统重新启动并开始对准 |
| **4** | 位置清零 | 将当前的北向/东向位置增量归零 |

**控制示例 (Terminal):**
```bash
# 开启 DVL (入水后执行)
ros2 topic pub --once /zit6/cmd/ins_command std_msgs/msg/UInt8 "{data: 1}"

# 位置清零
ros2 topic pub --once /zit6/cmd/ins_command std_msgs/msg/UInt8 "{data: 4}"
```

> [!CAUTION]
> **严禁在空气中执行开启 DVL 指令 (data: 1)**，否则会造成换能器物理损坏！
