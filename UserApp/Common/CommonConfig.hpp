#ifndef __COMMON_CONFIG_HPP
#define __COMMON_CONFIG_HPP

#include <stdint.h>

namespace auv {

/**
 * @brief 4-DOF 导航状态结构体 (X, Y, Z, Yaw)
 */
struct NavState {
    float x;
    float y;
    float z;
    float yaw;

    float vx;
    float vy;
    float vz;
    float vyaw;

    uint8_t imu_state;
    uint8_t dvl_state;
    uint32_t timestamp;
};

/**
 * @brief 控制注入层级
 */
enum class ControlLevel : uint8_t {
    POSITION = 0,
    VELOCITY = 1,
    ACTUATOR = 2
};

/**
 * @brief Offboard 目标值结构体
 */
struct OffboardSetpoint {
    ControlLevel level;
    float data[4];      // [X, Y, Z, Yaw]
    uint32_t type_mask;
};

/**
 * @brief 系统常量定义
 */
struct Constants {
    static constexpr float CONTROL_FREQ = 50.0f;
    static constexpr uint32_t CONTROL_PERIOD_MS = 20;
    static constexpr float DEG2RAD = 0.0174532925f;
    static constexpr float RAD2DEG = 57.2957795f;
};

} // namespace auv

#endif
