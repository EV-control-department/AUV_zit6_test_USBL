#include "IICTask.hpp"
#include "GlobalContext.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SoftWatchdog.hpp"

void UserApp_IICTask(void *argument) {
    auv::device::depth_sensor.Init();
    
    for (;;) {
        if (auv::device::depth_sensor.is_connected) {
            if (auv::device::depth_sensor.Read()) {
                float d = 0.0f;
                auv::device::depth_sensor.Depth(&d);
                current_depth_z = d;
                auv::device::SoftWatchdog::getInstance().feed(auv::device::SoftWatchdog::Component::DEPTH);
            }
        } else {
            // Retry init if not connected
            auv::device::depth_sensor.Init();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz sampling
    }
}
