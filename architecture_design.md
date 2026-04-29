这份架构设计文档的基础非常扎实。为了将我们之前深入探讨的**“内置运动学平滑器”、“前馈控制”、“无扰切换（特别是Z轴积分继承）”**以及**“微积分变频拆分协议”**完美融合进去，我们需要对 **第 3、4、5、6 节** 进行重构。

我将纯 C++ 类设计的职责划分得更加明确，将控制算法与 ROS 2 通信解耦。以下是为你注入了全部高级逻辑的完整 Markdown 文档，你可以直接覆盖原文件：

***

# AUV 固件 C++ 架构设计方案 (Optimized & Feedforward Ready)

本文档基于现有 C 语言工程封装经验与现代多旋翼/AUV控制理论，设计了 UserApp 下的模块化 C++ 架构，旨在实现逻辑解耦、前馈轨迹跟踪与极致的代码复用。

## 1. Porting (硬件适配层)
负责硬件外设的底层驱动与数据搬运。

* **`UartDmaTransport`**: 
    * 专用于 micro-ROS (USART2)，处理 Cache 一致性及 ORE 自愈。
* **`SerialPort`**: 
    * 通用串口 DMA 类，用于 **UART7 (INS)** 和 **UART4 (VIT6)**。
    * 支持循环缓冲读取，减少 CPU 中断频率。

## 2. Peripherals (外设驱动与模型层)
负责协议解析与物理特性建模。

* **`INS_Driver`**:
    * **职责**：封装原 `ImuSolve` 逻辑，解析惯导协议包。
    * **输出**：更新标准的 `NavState` 结构体（含全局位姿与机体系瞬态速度）。
* **`VIT6_Link`**:
    * **职责**：执行下位机动态定长帧协议（ID 0x01, 0x02, 0x03），包含 CRC-16 校验。
    * **控制功能**：高频下发推力矩阵（将上位机 -1.0~1.0 归一化浮点映射为 -1000~1000 整型）与舵机角度。
    * **配置功能**：转发静态配置参数（推力曲线、舵机零位）与状态指令。


---

### 3. Chassis (数学与底盘控制层) - *[Core Algorithm]*
负责核心控制算法、时序轨迹生成与动力分配。

* **`KinematicProfileGenerator` (内置运动学平滑器)**:
    * **职责**：纯数学物理边界计算模块（实例化 4 次，对应 X, Y, Z, Yaw）。接收突变的目标设定点，以 100Hz 的频率进行内部迭代，生成符合物理定律（`max_velocity`, `max_acceleration`）的平滑过渡曲线。
    * **输出与前馈供给**：同步输出四自由度理想时序状态：$P_d, V_d, A_d$。**该模块不仅提供位置追踪的靶点，更是底层 `CascadeController` 前馈控制流的唯一数据源泉。**
* **`SetpointRouter` (设定点路由与状态机)**:
    * **职责**：拦截来自上位机的 `CompactSetpoint`，管理系统的“接管层级 (Level)”状态机。
    * **旁路恢复机制 (Bumpless Transfer)**：当控制权从低层级（如 VELOCITY）切回高级环（如 POSITION）时，负责：
        1. 强制对齐影子平滑器的初始状态与当前真实传感器状态（消除 $P$ 突变）。
        2. 清洗平滑器的速度/加速度历史缓存。
* **`CascadeController` (级联前馈控制器)**:
    * **职责**：执行 4-DOF 级联 PID 运算，并**深度融合平滑器提供的前馈量**。
    * **前馈注入方程**：
        * **速度环期望**：$V_{target} = PID_{pos}(P_d - P_{actual}) + \mathbf{V_d}$ （速度前馈）
        * **推力环期望**：$F_{target} = PID_{vel}(V_{target} - V_{actual}) + \mathbf{Mass \cdot A_d}$ （惯性前馈） $+ \mathbf{Drag \cdot V_d|V_d|}$ （阻力前馈）(惯性前馈和阻力前馈暂时用0,这个不好调整)
    * **Z轴积分继承 (Trim Pre-loading)**：配合 `SetpointRouter`，在层级切回瞬间，绝对禁止 Z 轴积分器清零，而是将上一周期的实际 Z 轴推力**反向代入** Z 轴积分器，完美继承悬停配平浮力，防止潜器“掉高”或“上窜”。
* **`CoordinateManager`**:
    * **职责**：提供机体系 (Body) 与世界系 (World) 之间的位姿变换。
* **`KinematicsSolver`**:
    * **职责**：基于多旋翼或四角对峙推力器拓扑结构，将 4-DOF 期望推力/力矩矩阵求解为各推进器的独立归一化推力，并执行电机饱和限幅处理。

## 4. Application (应用业务与调度层)
负责系统状态管理、各功能模块的实例化以及 ROS 2 代理调度。

* **`Navigator`**: 获取 `INS_Driver` 数据，进行坐标变换，维护全局唯一的高信噪比 `Actual_State` 上下文。
* **`UrosNode`**: 异步运行。负责 ROS 2 话题的高速收发（变频发布：Pose 50Hz, Vel 100Hz, Thrust 200Hz）。
* **`CommandInterpreter`**: 
    * 解析并校验 `/auv/compact_setpoint` 和 `/auv/cmd/arm`。
    * **安全校验**：实现 ARM 动作的“双重连续确认”（10Hz+ 且持续 2s）。

---

## 5. 任务线程与调度 (Task Scheduling)
系统基于 FreeRTOS 划分为三个核心任务，为匹配时序前馈轨迹，主控频率全面提升至 **100Hz**。

### 5.1 Control_Loop_Task (优先级: Highest, 频率: 100Hz)
这是系统的绝对核心实时任务，严格保证 10ms 执行周期，杜绝抖动。
* **工作流**：
    1.  **观测更新**：从 `Navigator` 获取最新位姿与速度。
    2.  **轨迹演进**：驱动 `KinematicProfileGenerator` 进行一步积分，产出当前 10ms 的“影子期望状态”。
    3.  **旁路与级联**：调用 `CascadeController`。根据挂起状态跳过不必要的环路，将影子状态和真实状态作差，并叠加加速度前馈。
    4.  **动力分配**：调用 `KinematicsSolver` 进行矩阵分配，实施推力曲线限幅。
    5.  **高速下发**：通过 `VIT6_Link` (UART4 DMA) 将指令透传至执行层。

### 5.2 UROS_Communication_Task (优先级: Normal, 频率: 异步)
负责所有外部通讯逻辑，采用 micro-ROS Executor 轮询。
* **上行**：依既定频率发布变频状态话题与影子轨迹遥测 (`/auv/state/shadow_trajectory`)。
* **下行**：实时拷贝收到的控制帧至 `CommandInterpreter`，并触发低频配置更新。

### 5.3 Monitor_Failsafe_Task (优先级: High, 频率: 10Hz)
独立于主控循环的看门狗任务，负责系统级监控与失控降级逻辑。

---

## 6. 系统运行工作流 (Operational Workflow)

1.  **INIT (初始化与同步)**：
    * 硬件自检 -> 建立 micro-ROS 连接 -> 同步 `rmw_uros_sync_session` -> 开启 10Hz 心跳。
2.  **ARMING (解锁校验序列)**：
    * **触发**：监听 `/auv/cmd/arm`。
    * **校验**：内置定时器启动，必须在 **2 秒内连续收到 $\ge 10Hz$** 的 True 指令。
    * **动作**：下发 VIT6 功率板解锁序列 (ID 0x02)，初始化所有 PID 积分器为 0。
3.  **OFFBOARD (动态轨迹追踪)**：
    * 常态化运行于 100Hz 控制流。
    * 无论上位机发送的设定点频率如何波动，内置平滑器始终保证推力输出的 $C^2$（加速度）连续性。
    * 若发生控制层级波动（如遥控器介入与退出），触发**积分器无扰继承逻辑**。
4.  **FAILSAFE (安全超时与降级保护)**：
    * **超时触发**：连续 **500ms** 未收到 `/auv/compact_setpoint` 心跳更新。
    * **降级响应 (Fallback)**：绝不维持过期的绝对坐标。系统将强制丢弃 POSITION 层级，自动跌落至 **VELOCITY 层级**，并将期望速度设为 `[0,0,0,0]`，实现水下就地悬停减速。
    * **终极断联 (Phase B)**：若上位机整体断联超 3 秒，锁定推力输出，向 VIT6 发送 DISARM 指令。
5.  **DISARMING (锁定序列)**：
    * 收到有效 DISARM 序列，平滑器目标置零，待动能耗尽后切断动力分配。