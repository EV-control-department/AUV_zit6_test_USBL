# AUV 系统通讯与控制协议 (Draft)

本文档定义了 H7 主控与惯导 (INS)、下位机 (VIT6) 以及上位机 (micro-ROS) 之间的核心数据结构与通讯协议。

## 1. 惯导数据结构 (NavState)

用于在系统内部传递标准化的导航状态信息。

```cpp
struct NavState {
    // 4-DOF 位置与姿态 (Vector)
    float x;    // Position X (m)
    float y;    // Position Y (m)
    float z;    // Position Z (m)
    float yaw;  // Attitude Yaw (rad)

    // 4-DOF 速度与角速度 (Speed)
    float vx;   // Linear Velocity X (m/s)
    float vy;   // Linear Velocity Y (m/s)
    float vz;   // Linear Velocity Z (m/s)
    float vyaw; // Angular Velocity Yaw (rad/s)

    uint8_t imu_state; // 导航状态位
    uint8_t dvl_state; // DVL 状态位
    uint32_t timestamp; // 系统时间戳 (ms)
};
```

## 2. 下位机通讯协议 (Subordinate_Link)

H7 与 VIT6 之间的串口通讯协议，采用固定长度帧格式。

### 2.1 帧格式
| 偏移 | 字节 | 描述 |
| :--- | :--- | :--- |
| 0 | 0xAA | 帧头 1 |
| 1 | 0x55 | 帧头 2 |
| 2 | ID | 帧类型 (0x01: 控制, 0x02: ARM/DISARM, 0x03: 配置) |
| 3-N | Data | 有效载荷 (0x01/0x02 为定长数据) |
| N+1 | Checksum | 累加和校验 (从 ID 开始到 Data 结束) |
| N+2 | 0x0D | 帧尾 |

- `Thruster_Force[0-5]`: 期望推力 (-1000 到 1000)。
- `Servo_Angle[0-N]`: 期望舵机角度 (通常为定值或控制变量)。

### 2.3 状态切换定义 (Payload ID: 0x02)
用于控制下位机 VIT6 的解锁状态：
- `Status`: `0x01` (ARM/解锁), `0x00` (DISARM/锁定)。
- **要求**：执行切换时需持续发送约 2 秒以确保生效。

### 2.4 配置量定义 (Payload ID: 0x03)
用于向 VIT6 下发不常变动的参数：
- **Sub-ID 0x01**: PID 参数下发。
- **Sub-ID 0x02**: 舵机零位 (Zero Position) 设定。
- **Sub-ID 0x03**: 推力曲线 (Thrust Curve) 数据表。

---

## 3. 控制注入逻辑 (Control Injection Logic)

H7 自身不具备自主规划模式，所有控制目标均来自外部 (micro-ROS)。系统根据指令的**层级 (Level)** 决定从级联 PID 的哪一级开始运行。

### 3.1 控制层级定义 (Control Levels)
接管必须以“层”为单位，一旦接管某一层，该层的所有 6 自由度分量必须完整下发。

| 层级名称 | 注入点 | 输入数据内容 | 说明 |
| :--- | :--- | :--- | :--- |
| **POSITION** | 位置环入口 | `x, y, z, yaw` | 最外层，控制绝对位置和航向 |
| **VELOCITY** | 速度环入口 | `vx, vy, vz, vyaw` | 控制平移速度和旋转速率 |
| **ACTUATOR** | 动力分配入口 | `f_x, f_y, f_z, t_z` | 直接给定力与力矩 |

### 3.2 级联计算流程
1.  **接收**：从 `/auv/offboard_setpoint` 获取 `Level` 和 `Values`。
2.  **路由**：
    - 若 `Level == VELOCITY`：跳过位置环计算，直接将 `Values` 作为速度环的目标值。
    - 级联运行 `VEL -> ATT -> RATE -> ACTUATOR`。
3.  **输出**：最终计算结果通过 `Subordinate_Link` 下发给 VIT6。

---

## 4. micro-ROS 接口详情

### 4.1 发布话题 (H7 -> 上位机)
- **`/auv/nav_state`** (`nav_msgs/Odometry`): 
    - 包含 INS 融合后的完整 6-DOF 位姿和速度。

### 4.2 订阅话题 (上位机 -> H7)
- **`/auv/offboard_setpoint`** (自定义类型):
    ```cpp
    uint8 level      # 层级 (0:POS, 1:VEL, 2:ACT)
    float32[4] data  # 对应层级的目标值 (详见 4.2.1)
    uint32 type_mask # 位掩码，用于忽略特定轴 (Bit=1 表示忽略该轴)
    ```

#### 4.2.1 data[4] 数据映射表 (PX4 风格)
为了保持索引的一致性，前三个分量始终对应平移轴 (X, Y, Z)，第四个分量对应旋转轴 (Yaw)。

| 层级 (Level) | data[0] (X) | data[1] (Y) | data[2] (Z) | data[3] (Yaw) | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **0: POS** | x (m) | y (m) | z (m) | yaw (rad) | 位置与航向目标 |
| **1: VEL** | vx (m/s) | vy (m/s) | vz (m/s) | vyaw (r/s) | 速度与速率目标 |
| **2: ACT** | F_x | F_y | F_z | T_z | 归一化推力与力矩 |

#### 4.2.2 type_mask 定义 (PX4 风格)
掩码位与 `data[i]` 的索引严格对应。

| 位 (Bit) | 对应数据 | 描述 |
| :--- | :--- | :--- |
| **0 (0x01)** | data[0] | 忽略 X 轴控制 (Position/Velocity/Force) |
| **1 (0x02)** | data[1] | 忽略 Y 轴控制 (Position/Velocity/Force) |
| **2 (0x04)** | data[2] | 忽略 Z 轴控制 (Position/Velocity/Force) |
| **3 (0x08)** | data[3] | 忽略 Yaw 轴控制 (Angle/Rate/Torque) |

*例如：若 `level=0 (POS)` 且 `type_mask=0x03` (忽略 X 和 Y)，则系统仅会进行高度 (Z) 和姿态的自动控制。*

> [!NOTE]
> 对于 **ATTITUDE** 层级，通常只需给定 `roll` 和 `pitch`，而 `yaw` 的控制往往在 `POSITION` 或 `VELOCITY` 层级已经处理。但在本协议中，为了保持层级完整性，每一层均要求提供完整的控制矢量。

---

### 4.1 发布话题 (H7 -> 上位机)
- **`/auv/nav_state`** (`nav_msgs/msg/Odometry`):
    - `pose.pose.position`: [x, y, z] (m)
    - `pose.pose.orientation`: 转换自 [roll=0, pitch=0, yaw] 的四元数
    - `twist.twist.linear`: [vx, vy, vz] (m/s)
    - `twist.twist.angular`: [0, 0, vyaw] (rad/s)
- **`/auv/heartbeat`** (`std_msgs/msg/UInt32`):
    - `data`: 系统状态掩码或运行时间戳 (ms)

### 4.2 订阅话题 (上位机 -> H7)
- **`/auv/offboard_setpoint`** (自定义类型 `auv_msgs/msg/OffboardSetpoint`):
    - `uint8 level`: 注入层级 (0:POS, 1:VEL, 2:ACT)
    - `float32[4] data`: [X, Y, Z, Yaw] 目标矢量
    - `uint32 type_mask`: 忽略位掩码
- **`/auv/config/pid`** (自定义类型 `auv_msgs/msg/PIDConfig`):
    - `uint8 axis`: 目标轴 (0:X, 1:Y, 2:Z, 3:Yaw)
    - `uint8 loop`: 目标环 (0:Position, 1:Velocity)
    - `float32 kp`, `ki`, `kd`: PID 参数
    - `float32 i_limit`, `out_limit`: 积分与输出限幅
- **`/auv/config/servo`** (自定义类型 `auv_msgs/msg/ServoConfig`):
    - `uint8 index`: 舵机编号
    - `int16 pwm_at_0`: 0度对应的 PWM 值
    - `int16 pwm_at_180`: 180度对应的 PWM 值
    - `float32 min_limit`, `max_limit`: 软限幅角度范围 (deg)
    - `float32 max_velocity`: 最大允许转动角速度 (deg/s)
- **`/auv/config/thrust_curve`** (自定义类型 `auv_msgs/msg/ThrustCurve`):
    - `uint8 index`: 推进器编号
    - `float32[5] pwm`: 5 个关键点 PWM 值
    - `float32[5] thrust`: 对应 5 个点的推力值 (kg/N)

---

## 5. 控制策略说明 (Control Strategy)

> [!IMPORTANT]
> **计算权责划分**：
> 1. **H7 主控**：负责高层逻辑。包括导航解算、级联 PID 计算、以及动力分配 (Force Allocation)。H7 输出的是每个推进器应有的“推力值”。
> 2. **VIT6 驱动**：负责底层执行。包括接收 H7 的推力指令，根据内部存储的 `Thrust Curve` 将推力换算为 `PWM`，并最终驱动电调。
> 3. **参数下发**：PID 参数、舵机零位、推力曲线等均由上位机通过 micro-ROS 发给 H7，再由 H7 转发给 VIT6 存储。

---

## 6. 时间同步机制 (Time Synchronization)

为了确保惯导、主控 (H7) 与上位机 (ROS 2) 之间的数据具有时间一致性，系统采用以下对时策略：

### 6.1 MCU 与 上位机 (micro-ROS) 对时
*   **同步协议**：使用 micro-ROS 提供的 `rmw_uros_sync_session` 功能。
*   **频率**：初始化时进行 1 次对时，运行期间每 10 秒进行一次校准。
*   **实现**：H7 会通过同步后的时钟填充发布消息的 `header.stamp`。

### 6.2 惯导 (INS) 与 MCU 对时
*   **映射逻辑**：H7 在接收到惯导数据包时，记录下接收时刻的本地同步时间（Epoch Time），从而计算出数据在 ROS 2 世界中的绝对时间。

### 6.3 心跳数据要求
*   **Heartbeat 内容**：发布 `/auv/heartbeat` 时，`data` 字段的高 8 位表示状态掩码（0 为正常），低 24 位表示系统自对时后的运行时间 (ms)。
