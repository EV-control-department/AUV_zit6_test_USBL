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
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <std_msgs/msg/u_int32.h>

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

// 静态实例化驱动，避免依赖 C 堆
static auv::INS_Driver ins_driver(&huart7, &huart3);
static auv::NavState current_nav_state;

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
    rcl_publisher_t nav_pub;
    rcl_publisher_t heartbeat_pub;
    geometry_msgs__msg__PoseStamped pose_msg;
    std_msgs__msg__UInt32 heartbeat_msg;

    enum State { WAITING_AGENT, AGENT_CONNECTED };
    State state = WAITING_AGENT;

    for (;;) {
        switch (state) {
            case WAITING_AGENT:
                if (RCL_RET_OK == rmw_uros_ping_agent(100, 1)) {
                    rclc_support_init(&support, 0, NULL, &rcl_allocator);
                    rclc_node_init_default(&node, "auv_stm32_node", "", &support);
                    rclc_publisher_init_default(
                        &nav_pub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
                        "/auv/nav_pose"
                    );
                    rclc_publisher_init_default(
                        &heartbeat_pub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
                        "/auv/heartbeat"
                    );
                    state = AGENT_CONNECTED;
                }
                break;

            case AGENT_CONNECTED:
                if (RCL_RET_OK != rmw_uros_ping_agent(100, 1)) {
                    state = WAITING_AGENT;
                    if (RCL_RET_OK != rcl_publisher_fini(&nav_pub, &node)) {
                    }
                    if (RCL_RET_OK != rcl_publisher_fini(&heartbeat_pub, &node)) {
                    }
                    if (RCL_RET_OK != rcl_node_fini(&node)) {
                    }
                    if (RCL_RET_OK != rclc_support_fini(&support)) {
                    }
                } else {
                    static constexpr char frame_id[] = "world";
                    pose_msg.header.frame_id.data = const_cast<char *>(frame_id);
                    pose_msg.header.frame_id.size = sizeof(frame_id) - 1;
                    pose_msg.header.frame_id.capacity = sizeof(frame_id);

                    pose_msg.header.stamp.sec = HAL_GetTick() / 1000;
                    pose_msg.header.stamp.nanosec = (HAL_GetTick() % 1000) * 1000000;

                    pose_msg.pose.position.x = current_nav_state.x;
                    pose_msg.pose.position.y = current_nav_state.y;
                    pose_msg.pose.position.z = current_nav_state.z;
                    
                    // Yaw to Quaternion (Simplified for 4-DOF)
                    float cy = std::cos(current_nav_state.yaw * 0.5f * auv::Constants::DEG2RAD);
                    float sy = std::sin(current_nav_state.yaw * 0.5f * auv::Constants::DEG2RAD);
                    pose_msg.pose.orientation.w = cy;
                    pose_msg.pose.orientation.x = 0;
                    pose_msg.pose.orientation.y = 0;
                    pose_msg.pose.orientation.z = sy;

                    if (RCL_RET_OK != rcl_publish(&nav_pub, &pose_msg, NULL)) {
                    }

                    heartbeat_msg.data = HAL_GetTick();
                    if (RCL_RET_OK != rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL)) {
                    }
                }
                break;
        }
        osDelay(20); // 50Hz
    }
}
