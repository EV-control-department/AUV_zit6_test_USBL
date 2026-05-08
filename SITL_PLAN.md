# STM32 单片机内置实时仿真引擎 (SITL) 实施方案

## 1. 核心目标
在 STM32 内部实现一个简化的动力学引擎，通过全局配置开关控制。当启用仿真且系统解锁（Arm）时，底盘控制器的输入将从“物理传感器”无缝切换为“仿真数据”，从而实现不带硬件的闭环逻辑验证。

## 2. 详细步骤规划

### 第一阶段：配置层修改 (Config & Scripts)
1. **修改 `config.json`**：
   - 在 `sys` 或 `control` 模块下新增 `use_simulation` (bool) 标志。
   - 新增 `sim_params` 结构，包含水下阻力系数、质量、推力增益等参数。
2. **更新 `scripts/gen_config.py`**：
   - 确保 Python 脚本能解析上述新字段并生成对应的 C++ 头文件 `SystemConfig.hpp` 中的结构体映射。

### 第二阶段：仿真引擎实现 (Simulation Engine)
1. **创建 `UserApp/Control/AuvSimulator.hpp/cpp`**：
   - 实现一个简单的二阶累加器模型：
     - $F_{net} = F_{thrust} * k - V_{current} * C_{drag}$
     - $V_{next} = V_{current} + (F_{net} / Mass) * dt$
     - $P_{next} = P_{current} + V_{next} * dt$
   - 提供 `reset(pos)` 接口用于对齐初始位姿。

### 第三阶段：逻辑注入 (ControlTask Integration)
1. **注入点一：`updateNavigation()`**：
   - 判断 `auv::config::sys_config.control.use_simulation`。
   - 若为 `true`：
     - 若系统刚解锁，调用 `sim.reset()` 将当前传感器位置设为仿真起点。
     - 旁路 `ins_driver` 的更新，直接将 `sim.getPosition()` 和 `sim.getVelocity()` 写入 `nav` 结构体。
2. **注入点二：`computeAndPublish()`**：
   - 若仿真开启，将 `chassis.update()` 计算出的 4 自由度推力矢量 `forces` 喂给仿真引擎的 `step()` 函数。
   - 维持原来的硬件推力输出逻辑（仿真时可以选择屏蔽物理电机输出，或保留以观察电机反应）。

## 3. 坐标系一致性保障
- 仿真引擎应作用于**原始惯导坐标系**（即应用我们之前写的 `arm_home_pos` 偏移之前的坐标）。
- 这样，当仿真引擎输出位置时，后续的驱动层 `use_offset_` 逻辑依然生效，ROS 端看到的坐标依然会归零，保持逻辑一致。

## 4. 调试与验证流程
1. 修改 `config.json` 设置 `use_simulation: true`。
2. 运行 `gen_config.py` 更新配置头文件。
3. 编译并烧录固件。
4. 使用 `config_gui` 或命令行解锁 AUV。
5. 通过 `ros2 topic echo /zit6/state/pos` 观察。如果即使 AUV 静止在桌面上，你给指令后坐标也开始随之变动，说明仿真成功运行。

---
*注：我将开始寻找 `scripts/gen_config.py` 并准备修改配置定义。*
