#include "AppMain.hpp"
#include "ControlTask.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp"
#include <string.h>
#include "SerialPort.hpp"
#include <cstdio>

using namespace auv::device;
using namespace auv::control;


void ControlTask::fillActualState(const auv::common::NavState &nav, float (&actual_p)[4], float (&actual_v)[4]) {
    actual_p[0] = nav.x;
    actual_p[1] = nav.y;
    actual_p[2] = nav.z;
    actual_p[3] = nav.yaw;

    actual_v[0] = nav.vx;
    actual_v[1] = nav.vy;
    actual_v[2] = nav.vz;
    actual_v[3] = nav.vyaw;
}

void ControlTask::run() {
    init();

    for (;;) {
        refreshHardwareWatchdogIfNeeded();

        const uint32_t now = HAL_GetTick();
        last_dt_ms = static_cast<float>(now - last_tick_);
        last_tick_ = now;

        auv::common::NavState nav = updateNavigation();
        handleArmState(nav, now);
        computeAndPublish(nav);

        // 周期性调试信息（非阻塞，通过 UART5 DMA，忙时丢弃）
        static uint32_t last_log_ms = 0;
        if (now - last_log_ms >= 1000) {
            last_log_ms = now;
            char dbgbuf[128];
            int n = std::snprintf(dbgbuf, sizeof(dbgbuf), "DBG t=%lu dt=%.1f z=%.2f armed=%d\r\n",
                                  (unsigned long)now, last_dt_ms, nav.z, is_system_armed ? 1 : 0);
            if (n > 0) {
                auv::porting::SerialPort::transmitDebug(reinterpret_cast<const uint8_t*>(dbgbuf), (uint16_t)(n > (int)sizeof(dbgbuf) ? (int)sizeof(dbgbuf) : n));
            }
        }

        vTaskDelayUntil(&last_wake_time_, pdMS_TO_TICKS(kLoopPeriodMs));
    }
}

void ControlTask::init() {
    memset(ins_rx_buffer, 0, sizeof(ins_rx_buffer));
    // 初始化 motor_tx_packet：保留/设置帧头帧尾和 id，清零有效载荷字段
    taskENTER_CRITICAL();
    motor_tx_packet.head[0] = 0xFA;
    motor_tx_packet.head[1] = 0xAF;
    motor_tx_packet.id = 0x01;
    motor_tx_packet.Fx = 0.0f;
    motor_tx_packet.Fy = 0.0f;
    motor_tx_packet.Fz = 0.0f;
    motor_tx_packet.Fyaw = 0.0f;
    motor_tx_packet.Fpitch = 0.0f;
    motor_tx_packet.Froll = 0.0f;
    motor_tx_packet.tail[0] = 0xFB;
    motor_tx_packet.tail[1] = 0xBF;
    taskEXIT_CRITICAL();

    ins_driver.init();
    SoftWatchdog::getInstance().init(auv::config::sys_config.soft_watchdog);

    // 简单测试：非阻塞地通过 UART5 发送一条调试信息（忙时丢弃）
    const char test_msg[] = "DEBUG: UART5 OK\r\n";
    auv::porting::SerialPort::transmitDebug(reinterpret_cast<const uint8_t*>(test_msg), sizeof(test_msg) - 1);

    last_wake_time_ = xTaskGetTickCount();
    last_tick_ = HAL_GetTick();
}

void ControlTask::refreshHardwareWatchdogIfNeeded() {
    if (SoftWatchdog::getInstance().check()) {
        HAL_IWDG_Refresh(&hiwdg1);
    }
}

auv::common::NavState ControlTask::updateNavigation() {
    auv::common::NavState nav = auv::shared::snapshotNavState();
    ins_driver.update(nav);

    // 根据配置选择是否使用独立的 MS5837 深度覆盖融合深度
    if (auv::config::sys_config.sensors.z_data_source == auv::config::ZDataSource::USE_MS5837_Z) {
        float depth_snapshot = 0.0f;
        taskENTER_CRITICAL();
        depth_snapshot = current_depth_z;
        taskEXIT_CRITICAL();

        nav.z = depth_snapshot;
    }

    taskENTER_CRITICAL();
    shared_nav_state = nav;
    taskEXIT_CRITICAL();

    return nav;
}

void ControlTask::setControlLevelNone(const auv::common::NavState &nav) {
    float actual_p[4];
    float actual_v[4];
    fillActualState(nav, actual_p, actual_v);
    chassis.setControlLevel(auv::common::ControlLevel::NONE, actual_p, actual_v);
}

void ControlTask::forceDisarmWithNeutralLevel(const auv::common::NavState &nav) {
    taskENTER_CRITICAL();
    is_system_armed = false;
    arm_heartbeat_count = 0;
    auv::device::ins_driver.clearHomeOffset(); // 失锁时恢复原始坐标系
    taskEXIT_CRITICAL();
    setControlLevelNone(nav);
}

void ControlTask::handleArmState(const auv::common::NavState &nav, uint32_t now) {
    taskENTER_CRITICAL();
    const bool armed_snapshot = is_system_armed;
    const uint32_t heartbeat_snapshot = last_arm_heartbeat_ms;
    const uint32_t heartbeat_count_snapshot = arm_heartbeat_count;
    const uint32_t arm_start_snapshot = arm_start_ms;
    taskEXIT_CRITICAL();

    if (armed_snapshot) {
        if (now - heartbeat_snapshot > kArmedHeartbeatTimeoutMs) {
            forceDisarmWithNeutralLevel(nav);
        }
        return;
    }

    if (chassis.getControlLevel() != auv::common::ControlLevel::NONE) {
        setControlLevelNone(nav);
    }

    if (heartbeat_count_snapshot >= kArmMinHeartbeatCount &&
        (now - arm_start_snapshot >= kArmMinDurationMs)) {
        taskENTER_CRITICAL();
        const uint32_t hbt_data = last_arm_heartbeat_data;
        taskEXIT_CRITICAL();

        if (hbt_data == kRemoteModeHeartbeatData || auv::shared::isNavigationValid(nav)) {
            taskENTER_CRITICAL();
            if (!is_system_armed) {
                // 如果是刚解锁，将当前原始坐标注入注入驱动作为“家”偏移
                // 注意：此时 nav 已经是经过偏移处理的，但在刚解锁瞬间，驱动层的 use_offset 还是 false
                // 所以拿到的 nav 是原始值。注入后，下一帧起所有 nav 都会减去这个值。
                auv::device::ins_driver.setHomeOffset(nav.x, nav.y, nav.z, nav.yaw);
                
                // 控制器目标设为 0 (因为漂移已经被驱动层抵消了)
                target_p[0] = 0.0f;
                target_p[1] = 0.0f;
                target_p[2] = 0.0f;
                target_p[3] = 0.0f;
            }
            is_system_armed = true;
            taskEXIT_CRITICAL();
        } else {
            taskENTER_CRITICAL();
            arm_heartbeat_count = 0;
            taskEXIT_CRITICAL();
        }
    }

    if (now - heartbeat_snapshot > kDisarmedHeartbeatTimeoutMs) {
        taskENTER_CRITICAL();
        arm_heartbeat_count = 0;
        taskEXIT_CRITICAL();
    }
}

void ControlTask::computeAndPublish(const auv::common::NavState &nav) {
    float actual_p[4];
    float actual_v[4];
    fillActualState(nav, actual_p, actual_v);

    float target_snapshot[4];
    taskENTER_CRITICAL();
    for (int i = 0; i < 4; ++i) {
        target_snapshot[i] = target_p[i];
    }
    taskEXIT_CRITICAL();

    auto forces = chassis.update(actual_p, actual_v, target_snapshot);

    taskENTER_CRITICAL();
    for (int i = 0; i < 4; ++i) {
        last_output_forces[i] = forces[i];
    }
    const bool armed = is_system_armed;
    taskEXIT_CRITICAL();

    if (armed) {
        motor_driver.publishThrust(forces[0], forces[1], forces[2], forces[3]);
    } else {
        motor_driver.publishThrust(0, 0, 0, 0);
    }
}

void UserApp_ControlTask(void *argument) {
    (void)argument;
    ControlTask runner;
    runner.run();
}
