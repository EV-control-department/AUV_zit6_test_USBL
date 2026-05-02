#include "AppMain.hpp"
#include "INS_Driver.hpp"
#include "ChassisManager.hpp"
#include "CommonConfig.hpp"
#include "MotionController_Driver.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

// micro-ROS headers
#include <rcutils/allocator.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>

// 包含自定义消息 (v5.0 规范)
#include <zit6_interfaces/msg/zit_setpoint.h>
#include <zit6_interfaces/msg/zit_status.h>

// --- 硬件抽象层接口 ---
extern "C" {
    bool cubemx_transport_open(struct uxrCustomTransport *transport);
    bool cubemx_transport_close(struct uxrCustomTransport *transport);
    size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode);
    size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);
    void *microros_allocate(size_t size, void *state);
    void microros_deallocate(void *ptr, void *state);
    void *microros_reallocate(void *ptr, size_t new_size, void *state);
    void *microros_zero_allocate(size_t number_of_elements, size_t size_t_of_element, void *state);

    void UserApp_ControlTask(void *argument);
    void UserApp_MicroRosTask(void *argument);
}

extern "C" UART_HandleTypeDef huart7;
extern "C" UART_HandleTypeDef huart3;
extern "C" UART_HandleTypeDef huart2;
extern "C" UART_HandleTypeDef huart6;

// 全局状态与控制量
static float target_p[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float last_output_forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float last_dt_ms = 0.0f;
static uint32_t last_received_seq = 0;

static auv::INS_Driver ins_driver(&huart7, &huart3);
static auv::control::ChassisManager chassis;
static auv::MotionController_Driver motor_driver(&huart6);

// --- 安全 ARM 状态机变量 ---
static bool is_system_armed = false;
static uint32_t arm_heartbeat_count = 0;
static uint32_t last_arm_heartbeat_ms = 0;
static uint32_t last_arm_heartbeat_data = 0;
static uint32_t arm_start_ms = 0;
static auv::NavState shared_nav_state{};

namespace {

bool isNavigationValid(const auv::NavState& nav) {
    return (nav.imu_state >= 3 && nav.dvl_state == 1);
}

static auv::NavState snapshotNavState() {
    auv::NavState nav;
    taskENTER_CRITICAL();
    nav = shared_nav_state;
    taskEXIT_CRITICAL();
    return nav;
}

// 订阅回调：ZitSetpoint (v5.0)
void onZitSetpoint(const void *msgin) {
    const auto *msg = (const zit6_interfaces__msg__ZitSetpoint *)msgin;
    // 始终更新序列号用于丢包检测；但非上锁状态下只允许 ACTUATOR/HEARTBEAT-free 状态查询
    last_received_seq = msg->seq;

    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
        !std::isfinite(msg->z) || !std::isfinite(msg->yaw)) {
        return;
    }

    if (!is_system_armed) {
        // 安全策略：未解锁时不接受任何会影响推进器的设定点
        return;
    }

    // 记录序列号用于丢包监控
    last_received_seq = msg->seq;

    uint32_t level = msg->control_key & 0x03; // bits 0-1
    bool is_body   = (msg->control_key & 0x10) != 0;
    bool is_inc    = (msg->control_key & 0x20) != 0;
    uint32_t mask  = msg->type_mask;
    float val[4]   = {msg->x, msg->y, msg->z, msg->yaw};

    auto nav = snapshotNavState();
    if ((level == 0 || level == 1) && !isNavigationValid(nav)) return;

    if (level == 2) { // FORCE
        float fx = val[0], fy = val[1];
        if (!is_body) auv::control::CoordinateManager::worldToBody(nav.yaw, val[0], val[1], fx, fy);
        float forces[4] = {fx, fy, val[2], val[3]};
        taskENTER_CRITICAL();
        chassis.setActuatorForces(forces);
        // 以实际传感器值对齐 control level（避免传入 target_p 作为实际参数）
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        chassis.setControlLevel(auv::ControlLevel::ACTUATOR, actual_p, actual_v);
        taskEXIT_CRITICAL();
    }
    else if (level == 1) { // VEL
        taskENTER_CRITICAL();
        for (int i = 0; i < 4; i++) target_p[i] = val[i];
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        chassis.setControlLevel(auv::ControlLevel::VELOCITY, actual_p, actual_v);
        taskEXIT_CRITICAL();
    }
    else if (level == 0) { // POS
        taskENTER_CRITICAL();
        if (is_body && is_inc) {
            float wx, wy;
            auv::control::CoordinateManager::bodyToWorld(nav.yaw, val[0], val[1], wx, wy);
            if (mask & 0x01) target_p[0] = nav.x + wx;
            if (mask & 0x02) target_p[1] = nav.y + wy;
            if (mask & 0x04) target_p[2] = nav.z + val[2];
            if (mask & 0x08) target_p[3] = nav.yaw + val[3];
        } else {
            for (int i = 0; i < 4; i++) if (mask & (1 << i)) target_p[i] = val[i];
        }
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        chassis.setControlLevel(auv::ControlLevel::POSITION, actual_p, actual_v);
        taskEXIT_CRITICAL();
    }
}

void onArmHeartbeat(const void *msgin) {
    const auto *msg = (const std_msgs__msg__UInt32 *)msgin;
    taskENTER_CRITICAL();
    last_arm_heartbeat_ms = HAL_GetTick();
    last_arm_heartbeat_data = msg->data;
    
    if (!is_system_armed) {
        // 正常的解锁心跳逻辑
        if (arm_heartbeat_count == 0) arm_start_ms = last_arm_heartbeat_ms;
        arm_heartbeat_count++;
    }
    taskEXIT_CRITICAL();
}

void onInsCommand(const void *msgin) {
    const auto *message = static_cast<const std_msgs__msg__UInt8 *>(msgin);
    if (message == nullptr) return;
    switch (message->data) {
        case 1: ins_driver.setDvlPower(true); break;
        case 2: ins_driver.setDvlPower(false); break;
        case 3: ins_driver.restart(); break;
        case 4: ins_driver.resetPosition(); break;
    }
}

} // namespace

void UserApp_ControlTask(void *argument) {
    ins_driver.init();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_tick = HAL_GetTick();
    for (;;) {
        uint32_t now = HAL_GetTick();
        last_dt_ms = (float)(now - last_tick);
        last_tick = now;
        auv::NavState nav = snapshotNavState();
        ins_driver.update(nav);
        taskENTER_CRITICAL();
        shared_nav_state = nav;
        taskEXIT_CRITICAL();

        // --- ARM 状态机逻辑 ---
        taskENTER_CRITICAL();
        bool armed_snapshot = is_system_armed;
        uint32_t heartbeat_snapshot = last_arm_heartbeat_ms;
        uint32_t heartbeat_count_snapshot = arm_heartbeat_count;
        uint32_t arm_start_snapshot = arm_start_ms;
        taskEXIT_CRITICAL();

        if (armed_snapshot) {
            // 上锁阈值：200ms 心跳丢失
            if (now - heartbeat_snapshot > 200) {
                taskENTER_CRITICAL();
                is_system_armed = false; arm_heartbeat_count = 0;
                taskEXIT_CRITICAL();
                float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw}, actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
                chassis.setControlLevel(auv::ControlLevel::NONE, actual_p, actual_v);
            }
        } else {
            // 确保未上锁时，控制能级始终为 NONE
            if (chassis.getControlLevel() != auv::ControlLevel::NONE) {
                float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw}, actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
                chassis.setControlLevel(auv::ControlLevel::NONE, actual_p, actual_v);
            }
            
            // 解锁条件：收到 >=10 次心跳且持续至少 1000ms（10Hz * 1s）
            if (heartbeat_count_snapshot >= 10 && (now - arm_start_snapshot >= 1000)) {
                taskENTER_CRITICAL();
                uint32_t hbt_data = last_arm_heartbeat_data;
                taskEXIT_CRITICAL();

                // 正常模式需要导航就绪；hbt_data == 3 为遥控模式，不需要导航就绪
                if (hbt_data == 3 || isNavigationValid(nav)) {
                    taskENTER_CRITICAL();
                    is_system_armed = true;
                    taskEXIT_CRITICAL();
                } else {
                    taskENTER_CRITICAL();
                    arm_heartbeat_count = 0;
                    taskEXIT_CRITICAL();
                }
            }
            // 心跳窗口：若最后一次心跳超过 600ms，则重置计数
            if (now - heartbeat_snapshot > 600) {
                taskENTER_CRITICAL();
                arm_heartbeat_count = 0;
                taskEXIT_CRITICAL();
            }
        }

        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw}, actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        float target_snapshot[4];
        taskENTER_CRITICAL();
        for (int i = 0; i < 4; i++) target_snapshot[i] = target_p[i];
        taskEXIT_CRITICAL();

        auto forces = chassis.update(actual_p, actual_v, target_snapshot);
        taskENTER_CRITICAL();
        for(int i=0; i<4; i++) last_output_forces[i] = forces[i];
        bool armed = is_system_armed;
        taskEXIT_CRITICAL();

        if (armed) {
            motor_driver.publishThrust(forces[0], forces[1], forces[2], forces[3]);
        } else {
            motor_driver.publishThrust(0, 0, 0, 0);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void UserApp_MicroRosTask(void *argument) {
    rmw_uros_set_custom_transport(true, (void *)&huart2, cubemx_transport_open, cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);
    rcutils_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate; allocator.deallocate = microros_deallocate; allocator.reallocate = microros_reallocate; allocator.zero_allocate = microros_zero_allocate;
    rcutils_set_default_allocator(&allocator);

    rcl_allocator_t rcl_allocator = rcl_get_default_allocator();
    rclc_support_t support; rcl_node_t node; rclc_executor_t executor;
    rcl_subscription_t setpoint_sub; rcl_subscription_t arm_sub; rcl_subscription_t ins_cmd_sub;
    rcl_publisher_t pos_pub; rcl_publisher_t vel_pub; rcl_publisher_t thr_pub; rcl_publisher_t zithbt_pub; rcl_publisher_t status_pub;

    std_msgs__msg__Float32MultiArray pos_fb_msg; std_msgs__msg__Float32MultiArray vel_fb_msg; std_msgs__msg__Float32MultiArray thr_fb_msg;
    zit6_interfaces__msg__ZitSetpoint setpoint_msg; zit6_interfaces__msg__ZitStatus status_msg;
    std_msgs__msg__UInt8 ins_cmd_msg; std_msgs__msg__UInt32 arm_msg; std_msgs__msg__UInt32 node_heartbeat_msg;

    static float pos_buf[4], vel_buf[4], thr_buf[4];
    enum State { WAITING_AGENT, AGENT_CONNECTED } state = WAITING_AGENT;
    uint32_t last_ping_tick = 0, last_pos_pub_tick = 0, last_vel_pub_tick = 0, last_thr_pub_tick = 0, last_status_pub_tick = 0, last_hbt_pub_tick = 0;

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();
        if (state == WAITING_AGENT) {
            if ((now_ms - last_ping_tick) >= 1000U) {
                last_ping_tick = now_ms;
                if (RCL_RET_OK == rmw_uros_ping_agent(10, 1)) {
                    rclc_support_init(&support, 0, NULL, &rcl_allocator);
                    rclc_node_init_default(&node, "auv_stm32_node", "", &support);
                    rmw_uros_sync_session(10);

                    zit6_interfaces__msg__ZitSetpoint__init(&setpoint_msg);
                    zit6_interfaces__msg__ZitStatus__init(&status_msg);
                    std_msgs__msg__Float32MultiArray__init(&pos_fb_msg);
                    pos_fb_msg.data.data = pos_buf; pos_fb_msg.data.size = 4; pos_fb_msg.data.capacity = 4;
                    std_msgs__msg__Float32MultiArray__init(&vel_fb_msg);
                    vel_fb_msg.data.data = vel_buf; vel_fb_msg.data.size = 4; vel_fb_msg.data.capacity = 4;
                    std_msgs__msg__Float32MultiArray__init(&thr_fb_msg);
                    thr_fb_msg.data.data = thr_buf; thr_fb_msg.data.size = 4; thr_fb_msg.data.capacity = 4;

                    std_msgs__msg__UInt32__init(&node_heartbeat_msg);
                    std_msgs__msg__UInt8__init(&ins_cmd_msg);

                    rclc_publisher_init_default(&pos_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/pos");
                    rclc_publisher_init_default(&vel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/vel");
                    rclc_publisher_init_default(&thr_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/thr");
                    rclc_publisher_init_default(&zithbt_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/zit6/state/zithbt");
                    rclc_publisher_init_default(&status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitStatus), "/zit6/state/status");

                    rclc_subscription_init_default(&setpoint_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitSetpoint), "/zit6/cmd/setpoint");
                    rclc_subscription_init_default(&arm_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/zit6/cmd/agxhbt");
                    rclc_subscription_init_default(&ins_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/zit6/cmd/ins");

                    rclc_executor_init(&executor, &support.context, 3, &rcl_allocator);
                    rclc_executor_add_subscription(&executor, &setpoint_sub, &setpoint_msg, &onZitSetpoint, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor, &arm_sub, &arm_msg, &onArmHeartbeat, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor, &ins_cmd_sub, &ins_cmd_msg, &onInsCommand, ON_NEW_DATA);
                    state = AGENT_CONNECTED;
                }
            }
        } else {
            if (RCL_RET_OK != rmw_uros_ping_agent(10, 1)) { state = WAITING_AGENT; }
            else {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
                if (now_ms - last_hbt_pub_tick >= 1000) {
                    last_hbt_pub_tick = now_ms; node_heartbeat_msg.data = now_ms;
                    rcl_publish(&zithbt_pub, &node_heartbeat_msg, NULL);
                }
                if (now_ms - last_vel_pub_tick >= 16) {
                    last_vel_pub_tick = now_ms; auto nav = snapshotNavState();
                    vel_buf[0] = nav.vx; vel_buf[1] = nav.vy; vel_buf[2] = nav.vz; vel_buf[3] = nav.vyaw;
                    rcl_publish(&vel_pub, &vel_fb_msg, NULL);
                }
                if (now_ms - last_thr_pub_tick >= 33) {
                    last_thr_pub_tick = now_ms;
                    taskENTER_CRITICAL();
                    for (int i = 0; i < 4; i++) thr_buf[i] = last_output_forces[i];
                    taskEXIT_CRITICAL();
                    rcl_publish(&thr_pub, &thr_fb_msg, NULL);
                }
                if (now_ms - last_pos_pub_tick >= 33) {
                    last_pos_pub_tick = now_ms; auto nav = snapshotNavState();
                    pos_buf[0] = nav.x; pos_buf[1] = nav.y; pos_buf[2] = nav.z; pos_buf[3] = nav.yaw;
                    rcl_publish(&pos_pub, &pos_fb_msg, NULL);
                }
                if (now_ms - last_status_pub_tick >= 100) {
                    last_status_pub_tick = now_ms; auto nav = snapshotNavState();
                    taskENTER_CRITICAL();
                    status_msg.is_armed = is_system_armed;
                    status_msg.control_level = (uint8_t)chassis.getControlLevel();
                    status_msg.navigation_ready = isNavigationValid(nav);
                    for(int i=0; i<4; i++) status_msg.forces[i] = last_output_forces[i];
                    status_msg.cycle_time_ms = last_dt_ms;
                    status_msg.battery_voltage = 24.1f; // 实际应接 ADC
                    status_msg.error_flags = 0;       // 实际应根据诊断结果置位
                    taskEXIT_CRITICAL();
                    rcl_publish(&status_pub, &status_msg, NULL);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
