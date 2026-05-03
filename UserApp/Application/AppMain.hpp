#ifndef __APP_MAIN_HPP
#define __APP_MAIN_HPP

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief micro-ROS 任务入口 (由 freertos.c 调用)
 */
void UserApp_MicroRosTask(void *argument);

/**
 * @brief 控制与导航任务入口 (由 freertos.c 调用)
 */
void UserApp_ControlTask(void *argument);

/**
 * @brief IIC传感器任务入口 (由 freertos.c 调用)
 */
void UserApp_IICTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
