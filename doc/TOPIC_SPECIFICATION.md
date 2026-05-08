# ZIT6 AUV 通讯协议规范 (2026.5.8)

本文件为 ZIT6 AUV 与嵌入式固件之间的通信规范（自定义消息），包含字段含义、单位、取值范围、推荐频率以及从上电到初始化再到解锁（arm）的完整流程说明。

---

**约定与术语**
- 消息时间基于 ROS 时间；若在裸机或无时钟环境下，请在消息中使用相对时间戳（ms）。
- 轴顺序均为 X（前）、Y（右）、Z（下，向海底为正）——明确坐标系以避免混淆。
- 角度单位：弧度（rad）；线速度：米每秒（m/s）；位置：米（m）；推力或力：牛顿（N），嵌入式内部也可使用归一化推力[-1, 1]表示实际输出百分比，规范中会标注。

---

## 1. 控制指令 (Command Topics)

上位机发送到嵌入式的所有控制指令都走此类话题。要求：消息应带时间戳、使用小端序浮点（IEEE754）。

- 话题：`/zit6/cmd/setpoint`
    - 类型：`zit6_interfaces/ZitSetpoint`
    - 推荐频率：10Hz - 50Hz（位置环可低点，速度/力环频率高点）
    - 描述：上位机直接下达控制目标（位置/速度/推力），支持轴掩码与控制字。

### 1.1 `ZitSetpoint` 消息定义与字段说明
- `uint8 control_key`：控制模式与标志位组合
    - 低2位（bits0-1）：模式（0=POS位置, 1=VEL速度, 2=FORCE推力, 3=保留）
    - Bit4 (0x10)：目标为机体系（body）坐标；未置位则为世界系（world）
    - Bit5 (0x20)：增量模式（表示 x/y/z/yaw 为相对量而非绝对量）。
        - 在机体系增量模式下，x/y 将基于当前航向角转换到世界系后再进行位置累加。

- `uint8 type_mask`：轴掩码（按位）
    - 1：X轴，2：Y轴，4：Z轴，8：Yaw（偏航）
    - 使用示例：mask=1|2 表示只控制 X 和 Y

- `float32 x, y, z`：对应轴的目标值
    - 若 `control_key` 模式为 POS：单位为米（m），数值为目标位置（世界/机体按 Bit4）
    - 若为 VEL：单位为 m/s
    - 若为 FORCE：单位为 牛顿（N），若固件使用归一化推力则范围 [-1.0, 1.0] 表示最大推力百分比

- `float32 yaw`：偏航目标
    - 单位为弧度（rad）；在 POS/VEL 模式下语义分别为目标角/角速度；在 FORCE 模式通常为力矩或角力，视实现而定

- `uint32 seq`（可选）：消息序列号，用于调试与丢包检测

注意：发送端应确保不发送 NaN/Inf。嵌入式在收到不合法数值时应拒绝并在 `/zit6/state/status` 中标注错误标志。

- 话题：`/zit6/cmd/ins`
    - 类型：`std_msgs/UInt8`
    - 描述：INS 指令 （1:开启dvl,2:关闭dvl,3:重启惯导,4.（待验证）装订经纬度）

- 话题：`/zit6/cmd/light`
    - 类型：`std_msgs/UInt8`
    - 描述：LED 灯控制指令。（1.红2.黄3.绿）

- 话题：`/zit6/cmd/servo`
    - 类型：`std_msgs/Float32`
    - 描述：舵机角度控制指令。（单位为rad）

- 话题：`/zit6/cmd/agxhbt`
    - 类型：`std_msgs/UInt32`
    - 描述：上位机心跳，用于解锁（ARM）逻辑。（保证有10Hz以上持续心跳 1:普通解锁（需要ins精校准完成） 2.（保留）3.推力解锁（无需ins）
---

## 2. 配置与控制服务 (Service Specifications)

服务提供同步的配置交互，适用于需要确认执行结果或获取复杂结构数据的场景。

- 服务：`/zit6/update_params`
    - 类型：`zit6_interfaces/UpdateParams`
    - 描述：全量或增量更新系统配置（JSON 格式）。
    - **请求字段**：`string json`。包含要更新的参数（如 `{"chassis":{"pid":{"pos":{"kp":0.02}}}}`）。
    - **响应字段**：`bool success`, `string message`。

- 服务：`/zit6/get_params`
    - 类型：`zit6_interfaces/GetParams`
    - 描述：按需查询系统当前参数。
    - **请求字段**：`string[] paths`。路径数组（如 `["chassis.pid.pos"]`（目前这个还在调试，可返回空，详细见examples.md），若为空则返回全量配置。
    - **响应字段**：`bool success`, `string config_json`。返回包含所请求路径的最小化 JSON。

---

## 3. 状态反馈 (State Topics)

嵌入式定期/事件触发地发布自身状态供上位机监控与决策。

- 话题：`/zit6/state/status`
    - 类型：`zit6_interfaces/ZitStatus`
    - 推荐频率：10Hz
    - 描述：核心状态汇总，必须包含安全与运行态信息。

- 话题（已弃用）：`/zit6/state/pid_status`
    - 类型：`zit6_interfaces/ZitPidStatus`
    - 推荐频率：1Hz
    - 描述：全轴 PID 与运动学规划器参数回传大包，用于直观确认系统当前的所有控制参数。

- 话题：`/zit6/state/zithbt`
    - 类型：`std_msgs/UInt32`
    - 推荐频率：10Hz
    - 描述：节点/链路心跳（暂且未使用上）

- 话题：`/zit6/state/zit_status`
    - `bool is_armed`：是否已解锁（允许执行推力输出）

    - `uint8 arm_mode`：解锁模式
        - 0: DEFAULT（默认模式，需导航就绪），3: REMOTE（遥控模式，绕过导航就绪检查）

    - `uint8 control_level`：当前控制层级
        - 0: NONE（无人控/安全停），1: POS（位置环），2: VEL（速度环），3: FORCE（推力环）

    - `uint8 ins_state`：惯导内部状态 (0:待机, 1:粗对准, 2:精对准, 3:SINS/GPS/DVL, 4:SINS/DVL, 5:MRU)

    - `bool navigation_ready`：导航准备就绪（惯导、DVL、水声或其他定位子系统健康并对准）

    - `float32 forces[4]`：四个推进器的实际推力或归一化输出
        - 单位：N（如为百分比则标注为 [-1,1]）

    - `float32 cycle_time_ms`：控制主循环耗时，单位毫秒（未启用，保留）

    - `float32 battery_voltage`：电池电压，单位伏（未启用，保留）

    - `uint32 error_flags`：位域错误/警告标识位（未启用）
        - Bit0: 强制停机（致命）
        - Bit1: 传感器故障（IMU/DVL）
        - Bit2: 电压异常
        - Bit3: 通讯超时
        - 其余位保留

- 话题：`/zit6/state/vel`
    - 类型：`Float32MultiArray`
    - 推荐频率：60Hz
    - 描述：机体系速度 [vx, vy, vz,vyaw]，单位 m/s,rad/s；
- 话题：`/zit6/state/pos`
    - 类型：`Float32MultiArray`
    - 推荐频率：30Hz
    - 描述：世界系位置 [x, y, z, yaw] m/s,rad/s

- 话题：`/zit6/state/thr`
    - 类型：`Float32MultiArray`
    - 推荐频率：30Hz
    - 描述：机体系归一化推力 [fx,fy,fz,fyaw]


## 3. 上电 → 初始化 → 解锁（ARM）完整流程

下面按时间顺序描述固件从上电到允许实际推力输出的完整、可审计流程，包含检查项、超时与示例消息交互。

步骤概览：
1. auv上电（Power-On）
2. 上位机（取决于谁连上zit6->uart2tty->usbhub->）启动dds agent(具体见examples.md)
3. 若有ins,下发/zit6/cmd/ins
4. ins校准完全后下发/zit6/cmd/setpoint
7. 任何突发情况停止心跳包下发即可

详细流程：

- 步骤 1 — 硬件上电（Power-On）
    - 发生：接通主电源或电池接入
    - 要点：硬件看门狗暂停直到初始化完成；供电电压监测（若低于阈值 e.g. 18V）进入安全停机

- 步骤 2 — Bootloader / 启动代码
    - Bootloader（若存在）完成固件校验与跳转
    - 进入固件后，早期异常处理模块订阅紧急停机源

- 步骤 3 — HAL 与外设初始化
    - 初始化顺序（建议）：时钟 -> NVIC -> GPIO -> UART/SPI/I2C -> ADC -> PWM（驱动） -> DMA -> 定时器
    - 初始化完成后应在日志或 `/zit6/state/status` 中写入 `boot_complete` 标志（可作为诊断项）

- 步骤 4 — 传感器自检与标定
    - IMU：自检、滤波器复位，读取温度/偏置
    - DVL（若有）：启动并等待首帧有效回传
    - 深度传感器/压力传感器：读取并检查量程
    - 计时：每个传感器自检超时（示例：IMU 500ms, DVL 2s）
    - 如果关键传感器自检失败，设置 `error_flags` 并禁止解锁

- 步骤 5 — 通信与 micro-ROS 启动
    - 初始化串口/以太网/UDP 传输
    - 启动 micro-ROS 客户端节点：注册 publisher/subscriber
    - 发布初始状态：`/zit6/state/status`（is_armed=false, navigation_ready=false, error_flags 表示初始化状态）

- 步骤 6 — 上位机与嵌入式握手
    - 上位机应发送心跳 `/zit6/cmd/agxhbt`（例如 10Hz）并等待嵌入式回复状态
    - 同步参数（例如最大推力、推进器映射）
    - 当传感器自检通过并且上位机与嵌入式链路健康时，将 `navigation_ready=true`

- 步骤 7 — 预解锁检查（Pre-Arm Checks）
    - 必要条件（全部满足才允许解锁）：
        - `battery_voltage` 在安全范围内（示例：24V ± 20%）
        - `navigation_ready == true`（惯导航向正常，DVL/深度有效）
        - `error_flags == 0`（无致命错误）
        - 推进器控制路径健康（PWM/ESC 初始化成功）
        - 紧急停止开关处于允许位置
    - 嵌入式在 `/zit6/state/status` 中将各项检查结果逐项列出（建议扩展字段或 diagnostic topic）

- 步骤 8 — 发起解锁（Arm）
    - 上位机发送解锁心跳：`/zit6/cmd/agxhbt`（包含特定序列号或校验码以防误触发）
    - 或者发送专门的解锁命令（例如在 `zit6_interfaces` 中追加 `ArmRequest` 消息，包含 `reason` 与 `nonce`）
    - 上位机同时应开始以低幅度发送 `ZitSetpoint`（例如全部为 0 的保持包）以建立期望通道

- 步骤 9 — 嵌入式验证并完成解锁
    - 验证上位机心跳序列与预检条件；若通过：
        - 将 `is_armed` 置为 true
        - 发布 `/zit6/state/status`（is_armed=true）并记录解锁时间戳
        - 将推进器输出从安全零值渐进至允许输出（平滑开环，避免瞬时冲击）
    - 若验证失败，拒绝解锁并在 `error_flags` 中写入原因

超时与回退策略：
- 若在任一阶段出现关键超时（例如传感器自检、上位机心跳丢失），固件应自动回到安全停机状态并将 `is_armed=false`。
- 若在解锁后短时内检测到通讯丢失或致命错误，应立即将 `is_armed=false` 并将推进器置 0。

示例解锁序列（时间线形式）：
1. t=0s 上电
2. t=0.2s 固件启动并发布初始状态（is_armed=false）
3. t=0.5s 传感器自检通过，navigation_ready=true
4. t=1.0s 上位机开始发送 `/zit6/cmd/agxhbt` (seq=100)
5. t=1.1s 上位机发送 `ZitSetpoint`（保持包）
6. t=1.2s 嵌入式验证通过，发布 is_armed=true

---

## 6. 示例消息（JSON 表示，便于调试）
- `ZitSetpoint` 示例（位置环，世界系）

```
{ "seq": 123, "control_key": 0, "type_mask": 1|2|4, "x": 1.25, "y": 0.0, "z": -2.0, "yaw": 0.0 }
```

- `ZitStatus` 示例

```
{ "is_armed": false, "control_level": 0, "navigation_ready": true, "forces": [0,0,0,0], "cycle_time_ms": 2.3, "battery_voltage": 24.1, "error_flags": 0 }
```

---

## 7. 扩展建议与注意事项
- 明确推进器编号与力向映射并在启动时广播（便于 RQT 可视化）
- 对关键主题启用日志记录级别以便回放与事故分析
- 在测试阶段用 `is_armed` 的仿真模式验证控制律，避免真实推进器输出

如需我把这些字段转换为具体的 ROS2 .msg 定义文件（包含注释与示例），或把 ARM/Pre-ARM 的检查项实现为诊断 topic/diagnostic_msgs，请告诉我，我可以继续生成具体代码与消息文件。
