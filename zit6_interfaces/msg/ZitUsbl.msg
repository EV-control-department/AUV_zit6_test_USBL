# USBL超短基线定位系统数据
# 对应话题: /zit6/state/usbl

# 基阵姿态 (角度, 单位: 度)
float32 roll
float32 pitch
float32 yaw

# 信标位置 (NED坐标系, 单位: m)
float32 beacon_north
float32 beacon_east
float32 beacon_depth

# 信号能量 (30k-37k间隔1k, 共8个频点)
uint8[8] energy

# 传感器状态标志位
uint8 sensor_status

# 当前导航模式
uint8 nav_mode

# 时间戳 (ms)
uint32 timestamp
