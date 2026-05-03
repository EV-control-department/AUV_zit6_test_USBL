#ifndef MOTION_CONTROLLER_DRIVER_HPP
#define MOTION_CONTROLLER_DRIVER_HPP

#include "usart.h"
#include <string.h>

namespace auv {
namespace device {

/**
 * @class MotionController_Driver
 * @brief 动力控制板驱动类
 * 
 * 职责：
 * 1. 封装与下位机单片机的通信协议（0xFA 0xAF ... 0xFB 0xBF）。
 * 2. 将 4-DOF 或 6-DOF 力指令下发给推力分配逻辑。
 */
class MotionController_Driver {
public:
    struct __attribute__((packed)) Packet {
        uint8_t head[2] = {0xFA, 0xAF};
        uint8_t id = 0x01; // CMD_NORMAL_MODE
        float Fx;
        float Fy;
        float Fz;
        float Fyaw;
        float Fpitch;
        float Froll;
        float Angle_servo = 0;
        uint8_t tail[2] = {0xFB, 0xBF};
    };

    MotionController_Driver(UART_HandleTypeDef* huart, Packet* ext_pkt) : huart_(huart), pkt_ptr_(ext_pkt) {}
    MotionController_Driver(UART_HandleTypeDef* huart) : huart_(huart), pkt_ptr_(&pkt_internal_) {}

    /**
     * @brief 下发推力指令
     * @param fx, fy, fz, fyaw 4-DOF 目标推力/力矩
     */
    void publishThrust(float fx, float fy, float fz, float fyaw, float fp = 0, float fr = 0) {
        pkt_ptr_->Fx = fx;
        pkt_ptr_->Fy = fy;
        pkt_ptr_->Fz = fz;
        pkt_ptr_->Fyaw = fyaw;
        pkt_ptr_->Fpitch = fp;
        pkt_ptr_->Froll = fr;

        // 使用 DMA 传输，先刷写 D-Cache 以确保 DMA 拿到最新数据
        SCB_CleanDCache_by_Addr((uint32_t*)pkt_ptr_, sizeof(Packet));
        HAL_UART_Transmit_DMA(huart_, (uint8_t*)pkt_ptr_, sizeof(Packet));
    }

private:
    UART_HandleTypeDef* huart_;
    Packet* pkt_ptr_;
    alignas(32) Packet pkt_internal_;
};

} // namespace device
} // namespace auv

#endif // MOTION_CONTROLLER_DRIVER_HPP
