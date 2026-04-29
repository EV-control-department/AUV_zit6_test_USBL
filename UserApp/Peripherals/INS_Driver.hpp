#ifndef __INS_DRIVER_HPP
#define __INS_DRIVER_HPP

#include "SerialPort.hpp"
#include "CommonConfig.hpp"
#include <string.h>

namespace auv {

class INS_Driver {
public:
    INS_Driver(UART_HandleTypeDef* rx_uart, UART_HandleTypeDef* tx_uart)
        : rx_port_(rx_uart, 512), tx_uart_(tx_uart) {}

    void init() {
        rx_port_.startReceive();
        // 初始化：位置增量清零
        resetPosition();
    }

    /**
     * @brief 发送控制指令给惯导
     * @param cmd_id 指令ID (02:位置清零, 03:DVL电源, 04:重启)
     * @param value 指令值 (如DVL开启为01, 关闭为00)
     */
    void sendCommand(uint8_t cmd_id, uint8_t value) {
        uint8_t cmd[14] = {0xFC, 0xCF, cmd_id, value, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD, 0xDF};
        cmd[11] = checkData(cmd, 11);
        HAL_UART_Transmit(tx_uart_, cmd, 14, 20);
    }

    void resetPosition() { sendCommand(0x02, 0x00); }
    void setDvlPower(bool on) { sendCommand(0x03, on ? 0x01 : 0x00); }
    void restart() { sendCommand(0x04, 0x00); }

    bool update(NavState& state) {
        uint8_t temp_buf[256];
        uint16_t len = rx_port_.read(temp_buf, 256);

        if (len > 0) {
            for (uint16_t i = 0; i < len; i++) {
                if (parseByte(temp_buf[i]) && validateFrame()) {
                    decodePacket(state);
                    return true;
                }
            }
        }
        return false;
    }

private:
    SerialPort rx_port_;
    UART_HandleTypeDef* tx_uart_;

    static constexpr uint16_t kMaxFrameSize = 256;
    static constexpr uint16_t kMinFrameSize = 132; // 需要访问到 buf[129]
    uint8_t packet_buf_[kMaxFrameSize] = {0};
    uint16_t frame_len_ = 0;

    uint8_t checkData(const uint8_t* data, uint8_t size) {
        uint8_t v = data[0];
        for (uint8_t i = 1; i < size; ++i) {
            v ^= data[i];
        }
        return v;
    }

    bool parseByte(uint8_t b) {
        // State 0: 等待 0xFA
        if (frame_len_ == 0) {
            if (b == 0xFA) {
                packet_buf_[frame_len_++] = b;
            }
            return false;  // 还没收到完整帧
        }
        
        // State 1: 等待 0xAF
        if (frame_len_ == 1) {
            if (b == 0xAF) {
                packet_buf_[frame_len_++] = b;
            } else if (b == 0xFA) {
                // 重新对齐（发现新帧头）
                packet_buf_[0] = 0xFA;
                frame_len_ = 1;
            } else {
                // 垃圾字节，复位
                frame_len_ = 0;
            }
            return false;  // 还没收到完整帧
        }
        
        // State >=2: 收集后续数据字节
        // 中途遇到新帧头，按同学工程逻辑重新对齐
        if (packet_buf_[frame_len_ - 1] == 0xFA && b == 0xAF) {
            packet_buf_[0] = 0xFA;
            packet_buf_[1] = 0xAF;
            frame_len_ = 2;
            return false;
        }
        
        if (frame_len_ >= kMaxFrameSize) {
            frame_len_ = 0;
            return false;
        }
        
        packet_buf_[frame_len_++] = b;
        
        // 检查是否收到完整帧（检测 0xFB 0xBF 尾）
        return frame_len_ >= 2 &&
               packet_buf_[frame_len_ - 2] == 0xFB &&
               packet_buf_[frame_len_ - 1] == 0xBF;
    }

    bool validateFrame() {
        if (frame_len_ < kMinFrameSize) {
            frame_len_ = 0;
            return false;
        }
        if (packet_buf_[0] != 0xFA || packet_buf_[1] != 0xAF) {
            frame_len_ = 0;
            return false;
        }
        // 室外GPS/SINS/DVL模式：校验和@130
        if (checkData(packet_buf_, 130) != packet_buf_[130]) {
            frame_len_ = 0;
            return false;
        }
        return true;
    }

    void decodePacket(NavState& state) {
        float raw_data[12];
        // 按照NAV-300室外GPS/SINS/DVL模式协议
        memcpy(&(raw_data[3]), packet_buf_ + 2, 12);   // @2-13: 横滚(roll), 俯仰(pitch), 航向(yaw)
        memcpy(&(raw_data[0]), packet_buf_ + 54, 8);   // @54-61: 北向位置(X), 东向位置(Y)
        memcpy(&(raw_data[2]), packet_buf_ + 62, 4);   // @62-65: 深度(Z) - 改正：之前错读@46

        state.y = raw_data[0];   // 北向位置
        state.x = -raw_data[1];  // 东向位置（取反）
        state.z = raw_data[2];   // 深度
        state.yaw = raw_data[5]; // 航向

        state.imu_state = packet_buf_[129];       // @129: 当前导航模式
        state.dvl_state = packet_buf_[115] >> 7;  // @115 bit7: DVL有效状态位
        state.timestamp = HAL_GetTick();

        frame_len_ = 0;
    }
};

} // namespace auv

#endif
