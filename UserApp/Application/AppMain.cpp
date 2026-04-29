#include "AppMain.hpp"
#include "INS_Driver.hpp"
#include "CommonConfig.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <cmath>

// micro-ROS headers
#include <rcutils/allocator.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>

namespace {

struct CompactSetpoint {
    uint8_t target_loop = 0;
    uint8_t type_mask = 0;
    float data[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool valid = false;
};

static CompactSetpoint latest_setpoint;

static void onCompactSetpoint(const void *msgin) {
    const auto *message = static_cast<const std_msgs__msg__Float32MultiArray *>(msgin);
    if (message == nullptr || message->data.size < 6) {
        return;
    }

    latest_setpoint.target_loop = static_cast<uint8_t>(message->data.data[0]);
    latest_setpoint.type_mask = static_cast<uint8_t>(message->data.data[1]);
    for (uint8_t index = 0; index < 4; ++index) {
        latest_setpoint.data[index] = message->data.data[index + 2];
    }
    latest_setpoint.valid = true;
}

} // namespace

// 引用硬件句柄
extern "C" UART_HandleTypeDef huart7;
extern "C" UART_HandleTypeDef huart3;

// 静态实例化驱动，必须在回调函数之前定义或声明
static auv::INS_Driver ins_driver(&huart7, &huart3);
static auv::NavState current_nav_state;

namespace {
static void onInsCommand(const void *msgin) {
    const auto *message = static_cast<const std_msgs__msg__UInt8 *>(msgin);
    if (message == nullptr) return;

    switch (message->data) {
        case 1: ins_driver.setDvlPower(true); break;
        case 2: ins_driver.setDvlPower(false); break;
        case 3: ins_driver.restart(); break;
        case 4: ins_driver.resetPosition(); break;
        default: break;
    }
}

} // namespace

// 引用 freertos.c 中的传输接口
extern "C" bool cubemx_transport_open(struct uxrCustomTransport *transport);
extern "C" bool cubemx_transport_close(struct uxrCustomTransport *transport);
extern "C" size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
extern "C" size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
extern "C" UART_HandleTypeDef huart2;
extern "C" void *microros_allocate(size_t size, void *state);
extern "C" void microros_deallocate(void *pointer, void *state);
extern "C" void *microros_reallocate(void *pointer, size_t size, void *state);
extern "C" void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

/**
 * @brief 导航与控制任务 (50Hz)
 */
void UserApp_ControlTask(void *argument) {
    ins_driver.init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

    for (;;) {
        // 1. 获取惯导数据
        if (ins_driver.update(current_nav_state)) {
            // 数据已更新
        }

        // 2. 这里后续会添加 PID 控制逻辑
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief micro-ROS 任务 (管理连接与数据发布)
 */
void UserApp_MicroRosTask(void *argument) {
    // 1. 设置自定义传输层 (HUART2)
    rmw_uros_set_custom_transport(
        true, (void *)&huart2,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read
    );

    rcutils_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate;
    allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;
    allocator.state = NULL;
    if (!rcutils_set_default_allocator(&allocator)) {
        while (1) {
        }
    }

    rcl_allocator_t rcl_allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_subscription_t setpoint_sub;
    rcl_subscription_t ins_cmd_sub;
    rclc_executor_t executor;
    nav_msgs__msg__Odometry nav_state_msg;
    rcl_publisher_t nav_state_pub;
    geometry_msgs__msg__TwistStamped velocity_msg;
    rcl_publisher_t velocity_pub;
    std_msgs__msg__UInt32 ins_info_msg;
    rcl_publisher_t ins_info_pub;
    rcl_publisher_t heartbeat_pub;
    std_msgs__msg__Float32MultiArray setpoint_msg;
    std_msgs__msg__UInt8 ins_cmd_msg;
    std_msgs__msg__UInt32 heartbeat_msg;

    enum State { WAITING_AGENT, AGENT_CONNECTED };
    State state = WAITING_AGENT;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);
    uint32_t last_ping_tick = 0;
    uint32_t last_pose_pub_tick = 0;
    uint32_t last_velocity_pub_tick = 0;
    uint32_t last_ins_info_pub_tick = 0;
    uint32_t last_heartbeat_pub_tick = 0;

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        switch (state) {
            case WAITING_AGENT:
                if ((now_ms - last_ping_tick) >= 1000U) {
                    last_ping_tick = now_ms;
                    if (RCL_RET_OK == rmw_uros_ping_agent(10, 1)) {
                    rclc_support_init(&support, 0, NULL, &rcl_allocator);
                    rclc_node_init_default(&node, "auv_stm32_node", "", &support);
                    nav_msgs__msg__Odometry__init(&nav_state_msg);
                    geometry_msgs__msg__TwistStamped__init(&velocity_msg);
                    std_msgs__msg__UInt32__init(&heartbeat_msg);
                    std_msgs__msg__UInt32__init(&ins_info_msg);
                    std_msgs__msg__Float32MultiArray__init(&setpoint_msg);
                    std_msgs__msg__UInt8__init(&ins_cmd_msg);
                    rclc_publisher_init_default(
                        &nav_state_pub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                        "/zit6/state/pose"
                    );
                    rclc_publisher_init_default(
                        &velocity_pub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
                        "/zit6/state/velocity"
                    );
                    rclc_publisher_init_default(
                        &ins_info_pub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
                        "/zit6/state/ins_info"
                    );
                    rclc_publisher_init_default(
                        &heartbeat_pub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
                        "/zit6/heartbeat"
                    );
                    rclc_subscription_init_default(
                        &setpoint_sub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                        "/zit6/compact_setpoint"
                    );
                    rclc_subscription_init_default(
                        &ins_cmd_sub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                        "/zit6/cmd/ins_command"
                    );
                    rclc_executor_init(&executor, &support.context, 2, &rcl_allocator);
                    rclc_executor_add_subscription(
                        &executor,
                        &setpoint_sub,
                        &setpoint_msg,
                        &onCompactSetpoint,
                        ON_NEW_DATA
                    );
                    rclc_executor_add_subscription(
                        &executor,
                        &ins_cmd_sub,
                        &ins_cmd_msg,
                        &onInsCommand,
                        ON_NEW_DATA
                    );
                    last_pose_pub_tick = now_ms;
                    last_velocity_pub_tick = now_ms;
                    last_ins_info_pub_tick = now_ms;
                    last_heartbeat_pub_tick = now_ms;
                    state = AGENT_CONNECTED;
                    }
                }
                break;

            case AGENT_CONNECTED:
                if ((now_ms - last_ping_tick) >= 1000U) {
                    last_ping_tick = now_ms;
                    if (RCL_RET_OK != rmw_uros_ping_agent(10, 1)) {
                        state = WAITING_AGENT;
                        if (RCL_RET_OK != rclc_executor_fini(&executor)) {
                        }
                        if (RCL_RET_OK != rcl_subscription_fini(&setpoint_sub, &node)) {
                        }
                        if (RCL_RET_OK != rcl_subscription_fini(&ins_cmd_sub, &node)) {
                        }
                        if (RCL_RET_OK != rcl_publisher_fini(&nav_state_pub, &node)) {
                        }
                        if (RCL_RET_OK != rcl_publisher_fini(&velocity_pub, &node)) {
                        }
                        if (RCL_RET_OK != rcl_publisher_fini(&ins_info_pub, &node)) {
                        }
                        if (RCL_RET_OK != rcl_publisher_fini(&heartbeat_pub, &node)) {
                        }
                        if (RCL_RET_OK != rcl_node_fini(&node)) {
                        }
                        if (RCL_RET_OK != rclc_support_fini(&support)) {
                        }
                        nav_msgs__msg__Odometry__fini(&nav_state_msg);
                        geometry_msgs__msg__TwistStamped__fini(&velocity_msg);
                        std_msgs__msg__Float32MultiArray__fini(&setpoint_msg);
                        std_msgs__msg__UInt8__fini(&ins_cmd_msg);
                        std_msgs__msg__UInt32__fini(&heartbeat_msg);
                        std_msgs__msg__UInt32__fini(&ins_info_msg);
                        break;
                    }
                }

                    static constexpr char nav_frame_id[] = "world";
                    static constexpr char child_frame_id[] = "base_link";
                    const uint32_t now_sec = now_ms / 1000U;
                    const uint32_t now_nsec = (now_ms % 1000U) * 1000000U;

                    nav_state_msg.header.frame_id.data = const_cast<char *>(nav_frame_id);
                    nav_state_msg.header.frame_id.size = sizeof(nav_frame_id) - 1;
                    nav_state_msg.header.frame_id.capacity = sizeof(nav_frame_id);
                    nav_state_msg.child_frame_id.data = const_cast<char *>(child_frame_id);
                    nav_state_msg.child_frame_id.size = sizeof(child_frame_id) - 1;
                    nav_state_msg.child_frame_id.capacity = sizeof(child_frame_id);
                    nav_state_msg.header.stamp.sec = now_sec;
                    nav_state_msg.header.stamp.nanosec = now_nsec;

                    velocity_msg.header.frame_id.data = const_cast<char *>(child_frame_id);
                    velocity_msg.header.frame_id.size = sizeof(child_frame_id) - 1;
                    velocity_msg.header.frame_id.capacity = sizeof(child_frame_id);
                    velocity_msg.header.stamp.sec = now_sec;
                    velocity_msg.header.stamp.nanosec = now_nsec;

                    nav_state_msg.pose.pose.position.x = current_nav_state.x;
                    nav_state_msg.pose.pose.position.y = current_nav_state.y;
                    nav_state_msg.pose.pose.position.z = current_nav_state.z;
                    nav_state_msg.pose.pose.orientation.w = cosf(current_nav_state.yaw * 0.5f * auv::Constants::DEG2RAD);
                    nav_state_msg.pose.pose.orientation.x = 0;
                    nav_state_msg.pose.pose.orientation.y = 0;
                    nav_state_msg.pose.pose.orientation.z = sinf(current_nav_state.yaw * 0.5f * auv::Constants::DEG2RAD);
                    nav_state_msg.twist.twist.linear.x = current_nav_state.vx;
                    nav_state_msg.twist.twist.linear.y = current_nav_state.vy;
                    nav_state_msg.twist.twist.linear.z = current_nav_state.vz;
                    nav_state_msg.twist.twist.angular.z = current_nav_state.vyaw;

                    velocity_msg.twist.linear.x = current_nav_state.vx;
                    velocity_msg.twist.linear.y = current_nav_state.vy;
                    velocity_msg.twist.linear.z = current_nav_state.vz;
                    velocity_msg.twist.angular.x = 0.0f;
                    velocity_msg.twist.angular.y = 0.0f;
                    velocity_msg.twist.angular.z = current_nav_state.vyaw;

                    if ((now_ms - last_pose_pub_tick) >= 20U) {
                        last_pose_pub_tick = now_ms;
                        if (RCL_RET_OK != rcl_publish(&nav_state_pub, &nav_state_msg, NULL)) {
                        }
                    }

                    if ((now_ms - last_velocity_pub_tick) >= 10U) {
                        last_velocity_pub_tick = now_ms;
                        if (RCL_RET_OK != rcl_publish(&velocity_pub, &velocity_msg, NULL)) {
                        }
                    }

                    if ((now_ms - last_ins_info_pub_tick) >= 50U) {
                        last_ins_info_pub_tick = now_ms;
                        ins_info_msg.data = (static_cast<uint32_t>(current_nav_state.imu_state) << 8) |
                                            static_cast<uint32_t>(current_nav_state.dvl_state);
                        if (RCL_RET_OK != rcl_publish(&ins_info_pub, &ins_info_msg, NULL)) {
                        }
                    }

                    if ((now_ms - last_heartbeat_pub_tick) >= 100U) {
                        last_heartbeat_pub_tick = now_ms;
                        heartbeat_msg.data = ((static_cast<uint32_t>(current_nav_state.imu_state) & 0xFFU) << 24) |
                                             (now_ms & 0x00FFFFFFU);
                        if (RCL_RET_OK != rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL)) {
                        }
                    }

                    if (RCL_RET_OK != rclc_executor_spin_some(&executor, 0)) {
                    }
                break;
        }
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
