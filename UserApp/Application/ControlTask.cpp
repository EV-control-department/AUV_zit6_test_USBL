#include "ControlTask.hpp"
#include "GlobalContext.hpp"
#include "FreeRTOS.h"
#include "task.h"
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
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_tick = HAL_GetTick();
    
    for (;;) {
        // --- 硬件喂狗 ---
        HAL_IWDG_Refresh(&hiwdg1);

        uint32_t now = HAL_GetTick();
        last_dt_ms = (float)(now - last_tick);
        last_tick = now;
        
        // 3. 读取惯导数据并更新共享状态
        auv::NavState nav = shared::snapshotNavState();
        ins_driver.update(nav);
        
        taskENTER_CRITICAL();
        shared_nav_state = nav;
        taskEXIT_CRITICAL();

        // 暂时不恢复 publishThrust，先观察数据是否恢复
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}
