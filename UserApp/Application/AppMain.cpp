#include "AppMain.hpp"
#include "INS_Driver.hpp"
#include "CommonConfig.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

// 静态实例化驱动
static auv::INS_Driver* ins_driver = nullptr;
static auv::NavState current_nav_state;

/**
 * @brief 导航与控制任务 (50Hz)
 */
void UserApp_ControlTask(void *argument) {
    ins_driver = new auv::INS_Driver(&huart7, &huart3);
    ins_driver->init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

    for (;;) {
        // 1. 获取惯导数据
        if (ins_driver->update(current_nav_state)) {
            // 数据已更新
        }

        // 2. 这里后续会添加 PID 控制逻辑
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief micro-ROS 任务
 */
void UserApp_MicroRosTask(void *argument) {
    // 暂时留空，等待后续 micro-ROS 节点的具体实现
    for (;;) {
        osDelay(100);
    }
}
