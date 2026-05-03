#include "ControlTask.hpp"
#include "GlobalContext.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SoftWatchdog.hpp"
#include <string.h>

using namespace auv::device;
using namespace auv::control;

void UserApp_ControlTask(void *argument) {
    // 1. 基础内存初始化
    memset(ins_rx_buffer, 0, sizeof(ins_rx_buffer));
    memset(&motor_tx_packet, 0, sizeof(motor_tx_packet));
    motor_tx_packet.head[0] = 0xFA;
    motor_tx_packet.head[1] = 0xAF;
    motor_tx_packet.id = 0x01;
    motor_tx_packet.tail[0] = 0xFB;
    motor_tx_packet.tail[1] = 0xBF;

    // 2. 驱动初始化
    ins_driver.init();
    SoftWatchdog::getInstance().init();
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_tick = HAL_GetTick();
    
    for (;;) {
        // --- 硬件喂狗 (受软件看门狗保护) ---
        if (SoftWatchdog::getInstance().check()) {
            HAL_IWDG_Refresh(&hiwdg1);
        }

        uint32_t now = HAL_GetTick();
        last_dt_ms = (float)(now - last_tick);
        last_tick = now;
        
        auv::NavState nav = shared::snapshotNavState();
        ins_driver.update(nav);
        
        // Inject MS5837 depth data as the Z-axis position
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

        if (armed_snapshot) {
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
            if (chassis.getControlLevel() != auv::ControlLevel::NONE) {
                float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw},
                      actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
                chassis.setControlLevel(auv::ControlLevel::NONE, actual_p, actual_v);
            }

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
