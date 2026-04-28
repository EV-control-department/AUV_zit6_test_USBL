/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
/* USER CODE END Variables */
/* Definitions for micro_ros_task */
osThreadId_t micro_ros_taskHandle;
const osThreadAttr_t micro_ros_task_attributes = {
  .name = "micro_ros_task",
  .stack_size = 5000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for hardware_bridge */
osThreadId_t hardware_bridgeHandle;
const osThreadAttr_t hardware_bridge_attributes = {
  .name = "hardware_bridge",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imu_queue */
osMessageQueueId_t imu_queueHandle;
const osMessageQueueAttr_t imu_queue_attributes = {
  .name = "imu_queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END FunctionPrototypes */

void StartMicroROSTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of imu_queue */
  imu_queueHandle = osMessageQueueNew (16, sizeof(uint32_t), &imu_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of micro_ros_task */
  micro_ros_taskHandle = osThreadNew(StartMicroROSTask, NULL, &micro_ros_task_attributes);

  /* creation of hardware_bridge */
  hardware_bridgeHandle = osThreadNew(StartTask02, NULL, &hardware_bridge_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMicroROSTask */
/**
  * @brief  Function implementing the micro_ros_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMicroROSTask */
/* USER CODE BEGIN 0 */
volatile int uros_debug_state = 0;
volatile int uros_debug_error = 0;
/* USER CODE END 0 */

typedef enum {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} uros_state_t;

void StartMicroROSTask(void *argument)
{
  /* USER CODE BEGIN StartMicroROSTask */
  uros_state_t state = WAITING_AGENT;
  
  // micro-ROS configuration
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read
  );

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rcl_publisher_t publisher;
  std_msgs__msg__Int32 msg;
  msg.data = 0;

  /* Infinite loop */
  for(;;)
  {
    switch (state) {
      case WAITING_AGENT:
        // 1. Check if agent is available
        if (RCL_RET_OK == rmw_uros_ping_agent(100, 1)) {
          state = AGENT_AVAILABLE;
        }
        break;

      case AGENT_AVAILABLE:
        // 2. Initialize entities
        if (RCL_RET_OK == rclc_support_init(&support, 0, NULL, &allocator)) {
          if (RCL_RET_OK == rclc_node_init_default(&node, "auv_stm32_node", "", &support)) {
            if (RCL_RET_OK == rclc_publisher_init_default(
              &publisher,
              &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
              "/auv/heartbeat")) 
            {
              state = AGENT_CONNECTED;
            } else {
              // Cleanup if publisher fails
              rcl_node_fini(&node);
              rclc_support_fini(&support);
              state = WAITING_AGENT;
            }
          } else {
            // Cleanup if node fails
            rclc_support_fini(&support);
            state = WAITING_AGENT;
          }
        } else {
          state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED:
        // 3. Publish and monitor health
        if (RCL_RET_OK != rcl_publish(&publisher, &msg, NULL)) {
          state = AGENT_DISCONNECTED;
        } else {
          msg.data++;
          // Periodic ping to verify agent still exists
          if (msg.data % 10 == 0) {
             if (RCL_RET_OK != rmw_uros_ping_agent(100, 1)) {
                state = AGENT_DISCONNECTED;
             }
          }
        }
        break;

      case AGENT_DISCONNECTED:
        // 4. Cleanup old entities
        rcl_publisher_fini(&publisher, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        state = WAITING_AGENT;
        break;
    }

    osDelay(100);
  }
  /* USER CODE END StartMicroROSTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the hardware_bridge thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

