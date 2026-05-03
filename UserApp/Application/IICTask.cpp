#include "IICTask.hpp"
#include "GlobalContext.hpp"

using namespace auv::device;
#include "FreeRTOS.h"
#include "task.h"

void UserApp_IICTask(void *argument) {
    depth_sensor.Init();
    
    for (;;) {
        if (depth_sensor.is_connected) {
            if (depth_sensor.Read()) {
                float d = 0.0f;
                depth_sensor.Depth(&d);
                current_depth_z = d;
            }
        } else {
            // Retry init if not connected
            depth_sensor.Init();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz sampling
    }
}
