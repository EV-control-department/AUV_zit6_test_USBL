#include "ControlTask.hpp"
#include "GlobalContext.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "CoordinateManager.hpp"
#include <cmath>
#include <cstring>
#include <new>

using namespace auv::device;
using namespace auv::control;

void UserApp_ControlTask(void *argument) {
    // 初始化 DMA 缓冲区内容 (因为它们位于 NOLOAD 段，启动时可能包含随机数据)
    memset(ins_rx_buffer, 0, sizeof(ins_rx_buffer));
    new (&motor_tx_packet) auv::device::MotionController_Driver::Packet(); 

    ins_driver.init();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_tick = HAL_GetTick();
    
    for (;;) {
        uint32_t now = HAL_GetTick();
        last_dt_ms = (float)(now - last_tick);
        last_tick = now;
        
        auv::NavState nav = shared::snapshotNavState();
        ins_driver.update(nav);
        
        // 注入 MS5837 深度数据作为 Z 轴位置
        nav.z = current_depth_z;

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

        // 喂硬件看门狗
        HAL_IWDG_Refresh(&hiwdg1);

        if (armed_snapshot) {
            // 上锁阈值：500ms 心跳丢失
            if (now - heartbeat_snapshot > 500) {
                taskENTER_CRITICAL();
                is_system_armed = false;
                arm_heartbeat_count = 0;
                taskEXIT_CRITICAL();
                float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw},
                      actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
                chassis.setControlLevel(auv::ControlLevel::NONE, actual_p, actual_v);
            }
        } else {
            // 确保未上锁时，控制能级始终为 NONE
            if (chassis.getControlLevel() != auv::ControlLevel::NONE) {
                float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw},
                      actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
                chassis.setControlLevel(auv::ControlLevel::NONE, actual_p, actual_v);
            }

            // 解锁条件：收到 >=10 次心跳且持续至少 1000ms
            if (heartbeat_count_snapshot >= 10 && (now - arm_start_snapshot >= 1000)) {
                taskENTER_CRITICAL();
                uint32_t hbt_data = last_arm_heartbeat_data;
                taskEXIT_CRITICAL();

                if (hbt_data == 3 || shared::isNavigationValid(nav)) {
                    taskENTER_CRITICAL();
                    is_system_armed = true;
                    taskEXIT_CRITICAL();
                } else {
                    taskENTER_CRITICAL();
                    arm_heartbeat_count = 0;
                    taskEXIT_CRITICAL();
                }
            }
            // 心跳窗口：若最后一次心跳超过 1000ms，则重置计数
            if (now - heartbeat_snapshot > 1000) {
                taskENTER_CRITICAL();
                arm_heartbeat_count = 0;
                taskEXIT_CRITICAL();
            }
        }

        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw},
              actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        float target_snapshot[4];
        taskENTER_CRITICAL();
        for (int i = 0; i < 4; i++) target_snapshot[i] = target_p[i];
        taskEXIT_CRITICAL();

        auto forces = chassis.update(actual_p, actual_v, target_snapshot);
        taskENTER_CRITICAL();
        for (int i = 0; i < 4; i++) last_output_forces[i] = forces[i];
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
