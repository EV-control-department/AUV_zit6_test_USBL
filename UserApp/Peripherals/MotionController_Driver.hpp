#ifndef MOTION_CONTROLLER_DRIVER_HPP
#define MOTION_CONTROLLER_DRIVER_HPP

#include "usart.h"
#include <string.h>

namespace auv {

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
    MotionController_Driver(UART_HandleTypeDef* huart) : huart_(huart) {}

    /**
     * @brief 下发推力指令
     * @param fx, fy, fz, fyaw 4-DOF 目标推力/力矩
     */
    void publishThrust(float fx, float fy, float fz, float fyaw, float fp = 0, float fr = 0) {
        // 改为阻塞式发送 (Polling)，耗时约 2.8ms，彻底规避 DMA 与 Cache 冲突
        pkt_.Fx = fx;
        pkt_.Fy = fy;
        pkt_.Fz = fz;
        pkt_.Fyaw = fyaw;
        pkt_.Fpitch = fp;
        pkt_.Froll = fr;

        HAL_UART_Transmit(huart_, (uint8_t*)&pkt_, sizeof(pkt_), 10);
    }

private:
    UART_HandleTypeDef* huart_;

    // 将协议包缓冲区改为成员变量，确保其在 DMA 传输期间生命周期有效
    // 使用 alignas(32) 确保地址对齐到 Cache Line，避免 Clean 操作影响相邻变量
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
    
    alignas(32) Packet pkt_;
};

} // namespace auv

#endif // MOTION_CONTROLLER_DRIVER_HPP
