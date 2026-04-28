# AUV 固件 C++ 架构设计方案 (Optimized)

本文档基于现有 C 语言工程封装经验，设计了 UserApp 下的模块化 C++ 架构，旨在实现逻辑解耦与代码复用。

## 1. Porting (硬件适配层)
负责硬件外设的底层驱动与数据搬运。

*   **`UartDmaTransport`**: 
    *   专用于 micro-ROS (USART2)，处理 Cache 一致性及 ORE 自愈。
*   **`SerialPort`**: 
    *   通用串口 DMA 类，用于 **UART7 (INS)** 和 **UART4 (VIT6)**。
    *   支持循环缓冲读取，减少 CPU 中断频率。

## 2. Peripherals (外设驱动与模型层)
负责协议解析与物理特性建模。

*   **`INS_Driver`**:
    *   **职责**：封装原 `ImuSolve` 逻辑，解析 0xFA 0xAF 协议包。
    *   **输出**：更新标准的 `NavState` 结构体。
*   **`VIT6_Link`**:
    *   **职责**：执行 `protocol.md` 定义的所有串口协议（ID 0x01, 0x02, 0x03）。
    *   **控制功能**：高频下发推力矩阵 (Thruster Force) 与舵机角度 (Servo Angle)。
    *   **配置功能**：转发来自 micro-ROS 的配置参数与状态指令（ARM/DISARM）。

## 3. Chassis (数学与底盘控制层)
负责核心控制算法与坐标系变换。

*   **`CoordinateManager`**:
    *   **职责**：封装原 `coordinate_system.h/c` 的矩阵运算。
    *   **功能**：提供机体系 (Body) 与世界系 (World) 之间的位姿变换（`base2world`, `world2base`）。
*   **`/auv/config/thrust_curve`** (自定义类型 `auv_msgs/msg/ThrustCurve`):
    - `uint8 index`: 推进器编号
    - `float32[5] pwm`: 5 个关键点 PWM 值
    - `float32[5] thrust`: 对应 5 个点的推力值 (kg/N)
*   **`/auv/cmd/arm`** (`std_msgs/msg/Bool`):
    - **逻辑要求**：必须以 $\ge 10Hz$ 频率持续发送 `True` 达 **2 秒** 方可解锁。
*   **`CascadeController`**:
    *   **组件**：针对 X, Y, Z, Yaw 的 4 轴级联 PID 控制器。
    *   **注入逻辑**：支持从 POSITION, VELOCITY, 或 ACTUATOR 层级选择性注入，且支持 `type_mask` 掩码。
*   **`KinematicsSolver`**:
    *   **职责**：根据 6 推进器“六角对峙”布局，将 4-DOF 期望力/矩分配给各推进器。

## 4. Application (应用业务与调度层)
负责系统状态管理、各功能模块的实例化以及核心任务调度。

*   **`Navigator`**: 50Hz 运行。整合 `INS_Driver` 数据，进行平滑过滤与坐标转换，输出最终 `NavState`。
*   **`UrosNode`**: 异步运行。负责 ROS 2 话题收发、心跳更新及系统对时。
*   **`OffboardHandler`**: 
    *   职责：解析 `OffboardSetpoint` 和 `cmd/arm` 指令。
    *   **核心逻辑**：实现“双重连续确认”安全检查（10Hz+, 2s）。

---

## 5. 任务线程与调度 (Task Scheduling)

系统基于 FreeRTOS 划分为三个核心任务，通过优先级确保控制的实时性。

### 5.1 Control_Loop_Task (优先级: High, 频率: 50Hz)
这是系统的核心实时任务，必须严格保证 20ms 的执行周期。
*   **工作流**：
    1.  **传感器获取**：从 `Navigator` 获取经过坐标转换后的最新位姿。
    2.  **目标获取**：从 `OffboardHandler` 读取最新的控制目标（带超时检测）。
    3.  **级联解算**：调用 `CascadeController`，根据 Level 注入点执行 PID 运算。
    4.  **动力分配**：调用 `KinematicsSolver` 将力/矩分配为 6 路原始推力。
    5.  **串口下发**：通过 `VIT6_Link` 以串口 DMA 方式下发控制帧。

### 5.2 UROS_Communication_Task (优先级: Normal, 频率: ~100Hz)
负责所有外部通讯逻辑，与控制环路异步运行。
*   **职责**：发布 `nav_state`, `heartbeat`；订阅调参和控制话题。

### 5.3 Monitor_Failsafe_Task (优先级: Low, 频率: 10Hz)
负责系统监控与异常处理（如惯导丢失或上位机离线保护）。

---

## 6. 系统运行工作流 (Operational Workflow)

1.  **INIT (初始化与同步)**：
    *   硬件自检 -> 建立 micro-ROS 连接 -> 开启心跳发布。
2.  **ARMING (解锁校验序列)**：
    *   **触发**：H7 开始监听 `/auv/cmd/arm` 话题。
    *   **校验**：H7 内部计时器启动，必须在 **2 秒内连续收到 10Hz 以上** 的 ARM=True 指令。
    *   **动作**：校验通过后，向 VIT6 发送解锁序列 (2秒 ID 0x02)，随后正式允许电机转动。
3.  **OFFBOARD (任务执行)**：
    *   进入 50Hz 闭环。若上位机心跳中断或 ARM 指令变为 False，立即切入保护逻辑。
4.  **DISARMING (锁定序列)**：
    *   收到 DISARM 指令（同样需满足持续性校验），H7 立即归零动力并向 VIT6 发送锁定指令。
5.  **FAILSAFE (安全保护三段式)**：
    *   **Phase A (紧急上浮)**：检测到上位机断联，强制执行 **5 秒** 向上推力（Fz > 0）。
    *   **Phase B (电机停止)**：完全锁死下发电机推力为零关闭输出，进入错误挂起状态。
